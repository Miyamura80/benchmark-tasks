"""Volume-conserving water model for a tipping bath and catch tray.

The reservoir uses a sloshing free surface in a rectangular cross-section.
Water crosses a finite lip as weir discharge, then travels as ballistic
parcels. Parcels are classified by where they hit the tray or bench.
"""

from dataclasses import dataclass
import math

import numpy as np


WATER = (0.21, 0.68, 0.80, 0.55)
SURFACE = (0.36, 0.84, 0.89, 0.68)
GLINT = (0.75, 0.94, 0.96, 0.76)


@dataclass
class Parcel:
    position: np.ndarray
    velocity: np.ndarray
    volume_ml: float


class BathFluid:
    gravity = 9.81
    width = 0.082
    half_x = 0.038
    height = 0.048
    center = np.array((-0.050, 0.068, 0.0))
    catch = (0.137, -0.022, 0.068, 0.058, 0.217)
    lip_half_width = 0.020
    lip_depth = -0.0015

    def __init__(self, pivot):
        self.pivot = np.asarray(pivot, dtype=float)
        self.cup_ml = 107.5
        self.basket_ml = 2.5
        self.caught_ml = 0.0
        self.spill_ml = 0.0
        self.pending_ml = 0.0
        self.parcels = []
        self.slosh = 0.0
        self.slosh_rate = 0.0
        self.last_rate = 0.0
        self.tilt = 0.0
        self.flow_ml_s = 0.0
        self.clock = 0.0
        self.emit_index = 0
        self.ripples = []

    def normal(self):
        angle = self.tilt + self.slosh
        return -math.sin(angle), math.cos(angle)

    def surface(self):
        """Invert the projected volume CDF of a uniformly filled box."""
        nx, nz = self.normal()
        spans = sorted((2*self.half_x*abs(nx), self.height*abs(nz)))
        short, long = spans
        minimum = -self.half_x*abs(nx) + min(-self.height*nz, 0.0)
        fraction = min(1.0, max(0.0, self.cup_ml*1e-6/(2*self.half_x*self.width*self.height)))
        if short < 1e-8:
            projected = long*fraction
        elif fraction < short/(2*long):
            projected = math.sqrt(2*short*long*fraction)
        elif fraction < 1-short/(2*long):
            projected = long*fraction+0.5*short
        else:
            projected = short+long-math.sqrt(2*short*long*(1-fraction))
        return minimum + projected

    def rotation(self):
        co, si = math.cos(self.tilt), math.sin(self.tilt)
        return np.array(((co, 0, si), (0, 1, 0), (-si, 0, co)))

    def to_world(self, point, rotation):
        return self.pivot + rotation @ (self.center + point)

    def polygon(self):
        nx, nz = self.normal()
        plane = self.surface()
        corners = [(-self.half_x, -self.height), (self.half_x, -self.height),
                   (self.half_x, 0.0), (-self.half_x, 0.0)]
        inside, crossings = [], []
        for a, b in zip(corners, corners[1:]+corners[:1]):
            da, db = nx*a[0]+nz*a[1]-plane, nx*b[0]+nz*b[1]-plane
            if da <= 0:
                inside.append(a)
            if (da <= 0) != (db <= 0):
                u = da/(da-db)
                point = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
                inside.append(point)
                crossings.append(point)
        return inside, crossings

    def weight_torque(self):
        # Approximate liquid centre of mass along the bowl's x axis. A full
        # chamber stays near the centre; the shallow tilted pool moves downhill.
        fraction = self.cup_ml*1e-6/(2*self.half_x*self.width*self.height)
        x = self.center[0] + self.half_x*0.68*math.sin(self.tilt)*(1-fraction)
        z = -0.032
        lever_arm = x*math.cos(self.tilt) + z*math.sin(self.tilt)
        return lever_arm*(self.cup_ml*0.001)*self.gravity

    @property
    def in_flight_ml(self):
        return self.pending_ml + sum(p.volume_ml for p in self.parcels)

    @property
    def total_ml(self):
        return self.cup_ml+self.basket_ml+self.caught_ml+self.spill_ml+self.in_flight_ml

    def water_basin_top(self):
        _, _, sx, sy, floor = self.catch
        return floor + self.caught_ml*1e-6/(4*sx*sy)

    def advance(self, dt, tilt, rate, basket_lift):
        self.clock += dt
        self.tilt = tilt
        acceleration = float(np.clip((rate-self.last_rate)/dt, -24.0, 24.0))
        self.last_rate = rate
        self.slosh_rate += dt*(-90*self.slosh-6.5*self.slosh_rate-0.39*acceleration)
        self.slosh += self.slosh_rate*dt
        self.ripples = [r for r in self.ripples if self.clock-r[0] < 0.62]

        if basket_lift > 0.024 and self.basket_ml > 0:
            drainage = min(self.basket_ml, (4.5*self.basket_ml+0.18)*dt)
            self.basket_ml -= drainage
            self.cup_ml += drainage

        nx, nz = self.normal()
        lip_head = max(0.0, self.surface()-nx*self.half_x-nz*self.lip_depth)
        weir = (2/3)*0.65*(2*self.lip_half_width)*math.sqrt(2*self.gravity)
        self.flow_ml_s = weir*lip_head**1.5*1e6 if self.cup_ml > 0 else 0.0
        discharged = min(self.cup_ml, self.flow_ml_s*dt)
        self.cup_ml -= discharged
        self.pending_ml += discharged
        if self.pending_ml >= 0.30 or (self.pending_ml > 0 and self.flow_ml_s < 1.0):
            self.emit(lip_head)

        cx, cy, sx, sy, floor = self.catch
        active = []
        for parcel in self.parcels:
            previous = parcel.position.copy()
            parcel.velocity[2] -= self.gravity*dt
            parcel.position += parcel.velocity*dt
            height = self.water_basin_top()
            if parcel.position[2] > height:
                active.append(parcel)
                continue
            if previous[2] >= height:
                blend = (previous[2]-height)/(previous[2]-parcel.position[2])
                x, y, _ = previous+blend*(parcel.position-previous)
                if abs(x-cx) <= sx and abs(y-cy) <= sy:
                    self.caught_ml += parcel.volume_ml
                    if self.emit_index % 3 == 0 and len(self.ripples) < 12:
                        self.ripples.append((self.clock, float(x), float(y)))
                    continue
            if parcel.position[2] > floor-0.020:
                active.append(parcel)
            else:
                self.spill_ml += parcel.volume_ml
        self.parcels = active

    def emit(self, head):
        rotation = self.rotation()
        self.emit_index += 1
        local_y = 0.62*self.lip_half_width*math.sin(self.emit_index*2.39996)
        outlet = np.array((self.half_x + 0.004, local_y, self.lip_depth))
        position = self.to_world(outlet, rotation)
        speed = math.sqrt(2*self.gravity*head)
        direction = rotation @ np.array((0.49, 0, 0.87))
        velocity = speed*direction + np.array((0.09, 0.021*math.sin(self.emit_index), -0.040))
        self.parcels.append(Parcel(position, velocity, self.pending_ml))
        self.pending_ml = 0.0

    def draw(self, drawing):
        rotation = self.rotation()
        if self.cup_ml > 0.05:
            self.draw_pool(drawing, rotation)
        cx, cy, sx, sy, floor = self.catch
        pool = self.water_basin_top()-floor
        if pool > 0.00012:
            drawing.box((cx, cy, floor+pool/2), (sx, sy, pool/2), WATER)
            drawing.box((cx, cy, floor+pool), (sx-0.0004, sy-0.0004, 0.00025), SURFACE)
        for parcel in self.parcels:
            # Drop size scales with transported volume, elongated along its
            # velocity. Closely spaced streaks form a visible flowing sheet.
            radius = min(0.0036, max(0.0009, (parcel.volume_ml*2.4e-9)**(1/3)))
            tail = parcel.position - 0.023*parcel.velocity
            drawing.cord((tail, parcel.position), radius, SURFACE)
        for start, x, y in self.ripples:
            age = self.clock-start
            radius = 0.004+0.055*age
            points = [(x+radius*math.cos(t), y+radius*math.sin(t), floor+pool+0.0005)
                      for t in np.linspace(0, math.tau, 16)
                      if abs(x+radius*math.cos(t)-cx) < sx and abs(y+radius*math.sin(t)-cy) < sy]
            drawing.cord(points, 0.00032, (*GLINT[:3], max(0.0, 0.48*(1-age/0.62))))

    def draw_pool(self, drawing, rotation):
        nx, nz = self.normal()
        plane = self.surface()
        dx = 2*self.half_x/32
        for i in range(32):
            x = -self.half_x+(i+0.5)*dx
            if abs(nz) < 0.0001:
                low, high = (-self.height, 0) if nx*x <= plane else (0, 0)
            else:
                cut = (plane-nx*x)/nz
                cut = min(0.0, max(-self.height, cut))
                low, high = (-self.height, cut) if nz >= 0 else (cut, 0.0)
            if high-low > 0.0004:
                local = np.array((x, 0, 0.5*(low+high)))
                drawing.box(self.to_world(local, rotation),
                            (dx*0.501, self.width/2, 0.5*(high-low)), WATER, rotation)
        _, endpoints = self.polygon()
        if len(endpoints) == 2:
            a, b = (self.to_world(np.array((x, 0, z)), rotation) for x, z in endpoints)
            tangent = b-a
            length = np.linalg.norm(tangent)
            if length > 0.001:
                tangent /= length
                transverse = np.array((0, 1, 0))
                up = np.cross(tangent, transverse)
                drawing.box((a+b)*0.5, (length/2, self.width*0.499, 0.0003), SURFACE,
                            np.column_stack((tangent, transverse, up)))
