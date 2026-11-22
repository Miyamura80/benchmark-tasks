Vendored I2RT YAM model from the MuJoCo Menagerie `i2rt_yam` directory.

https://github.com/google-deepmind/mujoco_menagerie/tree/main/i2rt_yam

The original model and meshes are unmodified. `dual_yam.py` instantiates two copies,
sets their mounts and scripted control gains, and fits collision pads to the linear fingers.
The original model's license is included in this directory.
