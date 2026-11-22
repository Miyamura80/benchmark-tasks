"""Attach the mailer mechanics only to the embodiment which loaded this task."""

from contextvars import ContextVar
from functools import wraps
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import RLock

active_hold = ContextVar("prepared_mailer_active_hold", default=None)
build_lock = RLock()


def score(objects):
    hold = active_hold.get()
    return hold.score if hold is not None else 0.0


def success(objects):
    return score(objects) == 1.0


def is_mailer(env):
    return env.task is not None and getattr(env.task.module, "score", None) is score


def install():
    from yam_arena import inspect_embodiment as wrapper

    cls = wrapper.YamArenaEmbodiment
    if getattr(cls._task_step, "_prepared_mailer", False):
        return
    build, reset, task_step, close = (
        cls._ensure_sim,
        cls.reset,
        cls._task_step,
        cls.close,
    )

    @wraps(build)
    def ensure(env, goal):
        if not is_mailer(env) or env._env is not None:
            return build(env, goal)
        # Individual position jitter would tear apart the hinged assembly.
        env.object_jitter = 0.0
        # Configuring the scene requires a running Kit instance. Upstream
        # ordinarily opens it immediately before constructing the embodiment.
        with build_lock:
            if wrapper._APP is None:
                from isaaclab.app import AppLauncher
                import atexit
                import os

                os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
                wrapper._APP = AppLauncher(
                    headless=env.headless, device=env.device, enable_cameras=True
                ).app
                atexit.register(wrapper._close_app_at_exit)
            from .native import build_scene

            env._mailer_assets = TemporaryDirectory(prefix="prepared-mailer-")
            with build_scene(Path(env._mailer_assets.name), env.task):
                return build(env, goal)

    def detach(env):
        monitor = getattr(env, "_mailer_contacts", None)
        if monitor is not None:
            monitor.close()
            env._mailer_contacts = None

    @wraps(reset)
    def reset_episode(env, scene, *, seed=None):
        if not is_mailer(env):
            return reset(env, scene, seed=seed)
        detach(env)
        observation = reset(env, scene, seed=seed)
        from .native import Contacts

        env._mailer_contacts = Contacts(
            env, adhesive=getattr(env, "_mailer_adhesive", True)
        )
        return observation

    @wraps(task_step)
    def post_step(env):
        if not is_mailer(env):
            return task_step(env)
        monitor = env._mailer_contacts
        monitor.observe()
        token = active_hold.set(monitor.hold)
        try:
            return task_step(env)
        finally:
            active_hold.reset(token)

    @wraps(close)
    def close_episode(env):
        detach(env)
        try:
            return close(env)
        finally:
            assets = getattr(env, "_mailer_assets", None)
            if assets is not None:
                assets.cleanup()
                env._mailer_assets = None

    post_step._prepared_mailer = True
    cls._ensure_sim, cls.reset, cls._task_step, cls.close = (
        ensure,
        reset_episode,
        post_step,
        close_episode,
    )
