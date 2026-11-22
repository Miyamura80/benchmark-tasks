"""Simulate foundation transfer from a palette through a grasped foam sponge."""
from math import cos, pi, sin, sqrt

import mujoco as mj
import numpy as np

from appearance import complexion, gaussian, mesh, set_texture, texture
from fixture import NO_CONTACT, Fixture, ball, body, box, cylinder, slide, stand
from station import DT, element

CENTER = (.035, .037, .233)
FACE_X, FACE_Y = .083, .063
SHADE = np.array([.70, .47, .34, 1.])
DOCK = (-.024, .141, .207)
PALETTE = (.081, .141, .221)
PITCH = .008
TEXTURE_SIZE = 384


def height(x, y):
    base = .038 * sqrt(max(0., 1. - (x / FACE_X)**2 - (y / FACE_Y)**2))
    nose = .026 * np.exp(-(x / .027)**2 - (y / .011)**2)
    cheek = .007 * np.exp(-((x - .009) / .027)**2 - ((abs(y) - .033) / .015)**2)
    eye = .008 * np.exp(-((x + .029) / .012)**2 - ((abs(y) - .024) / .015)**2)
    return float(base + nose + cheek - eye)


def point(x, y, dz=0.):
    return (CENTER[0] + x, CENTER[1] + y, CENTER[2] + height(x, y) + dz)


def sculpt(x, y):
    side = abs(y)
    bridge = .0013 * gaussian(x, -.017, .016) * gaussian(side, 0., .007)
    alae = .0023 * gaussian(x, .014, .008) * gaussian(side, .010, .0035)
    nostrils = .0014 * gaussian(x, .019, .002) * gaussian(side, .0068, .0028)
    lids = .0026 * gaussian(x, -.031, .007) * gaussian(side, .029, .012)
    brows = .0021 * gaussian(x, -.044, .008) * gaussian(side, .028, .016)
    mouth = .0033 * gaussian(x, .051, .009) * gaussian(side, 0., .019)
    chin = .0029 * gaussian(x, .066, .008) * gaussian(side, 0., .025)
    return height(x, y) + bridge + alae - nostrils + lids + brows + mouth + chin


class Foundation(Fixture):
    title = "Blend foundation across the cheeks and nose"
    instruction = 'Pick up the makeup sponge and load it with foundation. Blend it across both cheeks and the nose of the silicone practice face secured to the workbench. Keep the lips and eyelids clean.'
    focus = (.020, .066, .263)

    def __init__(self, variant=0):
        super().__init__(variant)
        self.cells = np.array([(i * PITCH, j * PITCH)
            for i in range(-10, 11) for j in range(-7, 8)
            if (i * PITCH / FACE_X)**2 + (j * PITCH / FACE_Y)**2 <= .91])
        x, y = self.cells.T
        cheeks = (x >= -.008) & (x <= .032) & (abs(y) >= .016) & (abs(y) <= .048)
        self.regions = [cheeks & (y > 0), cheeks & (y < 0),
                        (abs(y) <= .008) & (x >= -.008) & (x <= .016)]
        self.target = np.logical_or.reduce(self.regions)
        self.protected = (((x >= -.040) & (x <= -.024) & (abs(y) >= .016) & (abs(y) <= .036))
                          | ((x >= .048) & (x <= .056) & (abs(y) <= .023)))
        self.coat = np.zeros(len(x))
        self.force_buffer = np.zeros(6)

    def asset_meshes(self, assets):
        n, rings = 96, 38
        vertices = [(0., 0., sculpt(0., 0.))]
        uv = [(.5, .5)]
        for i in range(1, rings + 1):
            radial = i / rings
            for j in range(n):
                theta = 2*pi*j/n
                x = FACE_X * radial * cos(theta)
                y = FACE_Y * radial * sin(theta)
                jaw = 1 - .26 * np.clip((x - .018) / .065, 0., 1.) * radial**4
                temple = 1 - .067 * gaussian(x, -.035, .019) * radial**4
                vertices.append((x, y * jaw * temple, sculpt(x, y)))
                uv.append(((y + FACE_Y) / (2 * FACE_Y), (x + FACE_X) / (2 * FACE_X)))
        faces = [(0, 1+j, 1+(j+1)%n) for j in range(n)]
        for i in range(rings-1):
            for j in range(n):
                a, b = 1+i*n+j, 1+i*n+(j+1)%n
                faces.extend(((a, a+n, b+n), (a, b+n, b)))
        floor = len(vertices)
        vertices.append((0., 0., -.028))
        uv.append((.5, .5))
        for j in range(n):
            a, b = 1+(rings-1)*n+j, 1+(rings-1)*n+(j+1)%n
            faces.append((a, floor, b))
        mesh(assets, "practice_face", vertices, faces, uv)
        texture(assets, "silicone_skin", TEXTURE_SIZE, (.81, .62, .52), .18, .25)

    def build(self, world):
        stand(world, xy=(.024, .018), half=(.153, .193))
        box(world, "face_mount", (CENTER[0], CENTER[1], .214), (.084,.065,.014), (.22,.26,.30,1))
        element(world, "geom", name="face_skin", type="mesh", mesh="practice_face",
                pos=CENTER, material="silicone_skin", **NO_CONTACT)
        for i, (x, y) in enumerate(self.cells):
            gx = (height(x + .0005, y)-height(x - .0005, y))/.001
            gy = (height(x, y + .0005)-height(x, y - .0005))/.001
            u = np.array([1., 0., gx])
            u /= np.linalg.norm(u)
            normal = np.array([-gx, -gy, 1.])
            normal /= np.linalg.norm(normal)
            box(world, f"skin_{i}", np.array(point(x,y)) - normal*.0016,
                (.00425,.00425,.0016), (1,1,1,0), xyaxes=(*u,*np.cross(normal,u)),
                friction=".7 .008 .001")
        for sign in (-1, 1):
            ball(world, f"outer_ear_{sign}", (CENTER[0]-.008, CENTER[1]+sign*.061, CENTER[2]+.006),
                 (.018, .009, .008), (.74, .50, .40, 1), **NO_CONTACT)
            ball(world, f"ear_cavity_{sign}", (CENTER[0]-.009, CENTER[1]+sign*.065, CENTER[2]+.011),
                 (.010, .0044, .003), (.60, .36, .30, 1), **NO_CONTACT)

        x,y,z = PALETTE
        cylinder(world, "palette_foot", (x,y,z-.012), (.034,.010), (.23,.15,.13,1))
        cylinder(world, "foundation_palette", (x,y,z-.001), (.028,.001), SHADE)
        box(world, "sponge_dock", DOCK, (.032,.036,.003), (.46,.54,.52,1))
        sponge = body(world, "sponge", (*DOCK[:2], DOCK[2]+.029))
        element(sponge, "freejoint", name="sponge_root")
        ball(sponge, "sponge_grasp", (0,0,.001), (.017,.016,.014), (.75,.24,.42,1),
             mass=.012, friction="2.6 .035 .004", solref=".009 1")
        pad = body(sponge, "foam", (0,0,-.014))
        slide(pad, "foam_compression", (0,0,1), (-.001,.009), stiffness="270", damping="1.5", springref="0")
        ball(pad, "sponge_pad", (0,0,-.003), (.020,.017,.010), (.88,.41,.56,1),
             mass=.008, friction=".6 .01 .001", solref=".007 1")

    def reset(self, s):
        self.coat[:] = 0
        self.supply = self.peak_force = self.contact_time = self.overforce_time = 0.
        self.pad_id = s.model.geom("sponge_pad").id
        self.palette_id = s.model.geom("foundation_palette").id
        self.skin_ids = {s.model.geom(f"skin_{i}").id for i in range(len(self.cells))}
        self.texture = s.model.texture("silicone_skin").id
        x = np.linspace(-FACE_X, FACE_X, TEXTURE_SIZE)[:, None]
        y = np.linspace(-FACE_Y, FACE_Y, TEXTURE_SIZE)[None, :]
        self.skin_rgb = complexion(x, y)
        set_texture(s.model, self.texture, self.skin_rgb)
        self.coating_grid=np.zeros((23,17))
        self.cell_indices=tuple((np.rint(self.cells/PITCH).astype(int)+[11,8]).T)
        def raster_axis(limit,span,midpoint):
            continuous=np.linspace(-limit,limit,TEXTURE_SIZE)/PITCH+midpoint
            low=np.clip(np.floor(continuous).astype(int),0,span-2)
            return low,np.clip(continuous-low,0.,1.)
        self.texture_x=raster_axis(FACE_X,23,11)
        self.texture_y=raster_axis(FACE_Y,17,8)

    def after_step(self, s):
        surface_id, load = None, 0.
        for i in range(s.data.ncon):
            c = s.data.contact[i]
            if c.geom1 == self.pad_id:
                other = c.geom2
            elif c.geom2 == self.pad_id:
                other = c.geom1
            else:
                continue
            if other != self.palette_id and other not in self.skin_ids:
                continue
            mj.mj_contactForce(s.model, s.data, i, self.force_buffer)
            if self.force_buffer[0] > load:
                surface_id, load = other, float(self.force_buffer[0])
        if load < .10:
            return
        if surface_id == self.palette_id:
            self.supply = min(1., self.supply + 3.5*DT)
            return
        self.peak_force = max(self.peak_force, load)
        self.contact_time += DT
        self.overforce_time += DT * (load > 5.5)
        if self.supply <= .01:
            return
        xy = s.data.geom_xpos[self.pad_id,:2] - CENTER[:2]
        radius = .0115 + .0035*min(load/2.5, 1.)
        footprint = np.maximum(0., 1.-np.sum((self.cells-xy)**2, axis=1)/radius**2)
        dose = DT * 13.5*min(self.supply*5,1.) * min(load/.4,1.) * footprint
        self.coat += dose*(1.-self.coat)
        self.supply = max(0., self.supply - float(np.sum(dose))*.0012)

    def measure(self, s):
        coated = self.coat >= .82
        coverage = float(np.mean(coated[self.target]))
        transfer = float(np.max(self.coat[self.protected]))
        left,right,nose = [float(np.mean(coated[r])) for r in self.regions]
        score = 1. if min(left,right,nose) >= .93 and transfer <= .2 and self.overforce_time < .15 else (
            .5 if coverage >= .55 and transfer <= .35 else .25 if coverage >= .25 and transfer <= .35 else 0.)
        return dict(score=score, coverage_pct=round(coverage*100,1),
                    protected_transfer_pct=round(transfer*100,1), peak_force_n=round(self.peak_force,2),
                    loaded_pct=round(self.supply*100,1), left_pct=round(left*100,1),
                    right_pct=round(right*100,1), nose_pct=round(nose*100,1),
                    overforce_s=round(self.overforce_time,3), face_contact_s=round(self.contact_time,3),
                    sponge_pos_m=s.position("sponge").round(4).tolist())

    def set_render_texture(self,s):
        self.coating_grid[self.cell_indices]=self.coat
        i,u=self.texture_x
        j,v=self.texture_y
        row=self.coating_grid[i[:,None],j[None,:]]*(1-v)+self.coating_grid[i[:,None],j[None,:]+1]*v
        next_row=self.coating_grid[i[:,None]+1,j[None,:]]*(1-v)+self.coating_grid[i[:,None]+1,j[None,:]+1]*v
        film=(row*(1-u[:,None])+next_row*u[:,None])*.64
        rgb=self.skin_rgb+film[:,:,None]*(SHADE[:3]-self.skin_rgb)
        set_texture(s.model, self.texture, rgb)
        return self.texture

    def demo(self, s, incomplete=False):
        dx,dy,dz = DOCK

        def move(arm,x,y,z,seconds,label,opening=None):
            s.move_arm(arm, (x,y,z), seconds, label, opening=opening, yaw=0.)

        def pick(arm):
            sx,sy,sz = s.position("sponge")
            move(arm,sx,sy,.335,.88,"Approach the foam sponge",.84)
            move(arm,sx,sy,sz,.80,"Lower the fingers around the sponge")
            move(arm,sx,sy,sz,.35,"Grip the foam sponge",.12)
            move(arm,sx,sy,.335,.75,"Lift the sponge from the dock")

        def put_down(arm):
            move(arm,dx,dy,.335,.74,"Carry the sponge back")
            move(arm,dx,dy,dz+.028,.78,"Set the sponge in its dock",.12)
            s.wait(.22)
            move(arm,dx,dy,dz+.028,.32,"Release the sponge",.92)
            move(arm,dx,dy,.335,.64,"Withdraw the gripper")

        def blend(arm, points, label, depth=.019):
            x,y=points[0]
            move(arm,*point(x,y,.059),.79,label)
            for x,y in points:
                target=np.array(point(x,y))
                tcp=s.data.site_xpos[s.sites[arm]]
                pad_xy=s.data.geom_xpos[self.pad_id,:2]
                goal_xy=target[:2]-np.clip(pad_xy-tcp[:2],-.018,.018)
                move(arm,*goal_xy,target[2]+.040,.31,label)
                move(arm,*goal_xy,target[2]+depth,.38,label)
                s.wait(.27)
                move(arm,*goal_xy,target[2]+.044,.29,label)
            move(arm,*point(x,y,.063),.52,"Lift the sponge clear")

        def cheek(arm,sign,label):
            blend(arm,[(x,sign*y) for y,xs in ((.019,(.002,.014,.027)),
                   (.033,(.027,.014,.002)),(.044,(.002,.014,.027))) for x in xs],label)

        pick("left")
        px,py,pz = PALETTE
        move("left",px,py,.340,.8,"Move over the open foundation palette")
        move("left",px,py,pz+.023,.74,"Load the applicator by contact")
        s.wait(.55)
        move("left",px,py,.341,.66,"Lift the loaded sponge")
        cheek("left",1,"Blend the first cheek")
        if not incomplete:
            blend("left",[(-.005,0),(.005,0),(.015,0),(.018,-.004),(.018,.004)],
                  "Blend along the nose",depth=.0235)
            cheek("left",-1,"Blend the second cheek")
        put_down("left")
        s.wait(.45)
