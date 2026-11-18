"""Render task materials from live fixture state, using the native YAM views."""

import math

import mujoco as mj
import numpy as np

from station import element, geom

METAL = (.52, .57, .60, 1)
DARK = (.075, .087, .10, 1)
INK = (.09, .12, .14, 1)
KRAFT = (.68, .53, .33, 1)
STOCK = (.93, .89, .78, 1)


def look(tree, world):
    assets = tree.find("asset")
    element(assets, "texture", name="bench_rubber", type="2d", builtin="checker",
            rgb1=".24 .286 .293", rgb2=".253 .303 .310", mark="random",
            markrgb=".34 .385 .39", random=".08", width=96, height=96)
    element(assets, "texture", name="finish_grit", type="2d", builtin="flat",
            rgb1=".52 .30 .15", rgb2=".52 .30 .15", mark="random",
            markrgb=".67 .45 .26", random=".42", width=256, height=256)
    element(assets, "material", name="workmat", texture="bench_rubber",
            texrepeat="9 9", specular=".15", shininess=".26")
    element(assets, "material", name="old_grit", texture="finish_grit",
            texrepeat="5 5", specular=".10", shininess=".18")
    for name, shine, spec in (("satin_metal", .62, .64), ("painted_shell", .34, .28),
                               ("soft_plastic", .18, .14)):
        element(assets, "material", name=name, specular=spec, shininess=shine)
    room = tree.find("worldbody")
    visual_box(room, "worktop", (-.03,0,.171), (.39,.43,.016), (.31,.34,.34,1),
               material="soft_plastic")
    visual_box(room, "bench_edge", (-.03,0,.154), (.39,.43,.004), (.15,.19,.21,1))
    for light in room.findall("light"):
        room.remove(light)
    element(room, "light", directional="true", pos="-.5 -.65 1.45", dir=".32 .53 -1",
            diffuse=".36 .34 .32", specular=".25 .23 .21", ambient=".22 .22 .22",
            castshadow="true")
    element(room, "light", directional="true", pos=".6 .5 1.1", dir="-.48 -.39 -1",
            diffuse=".43 .44 .46", specular=".17 .19 .22", castshadow="false")
    element(room, "light", directional="true", pos="-.3 .55 .8", dir=".2 -.7 -.55",
            diffuse=".15 .16 .17", castshadow="false")
    tree.find("visual/headlight").attrib.update(ambient=".19 .19 .19", diffuse=".24 .24 .24", specular=".12 .12 .12")
    tree.find("visual/quality").set("shadowsize", "2048")
    world.find("geom[@name='mat']").attrib.update(material="workmat", rgba="1 1 1 1")
    for part in world.iter("geom"):
        name = part.get("name", "")
        if any(word in name for word in ("shell", "cheek", "lid", "machine_base", "machine_arm", "machine_column")):
            part.set("material", "painted_shell")
        elif any(word in name for word in ("bearing", "pipe_stub", "lead_screw", "needle", "frame_", "flange", "tooth", "cog", "puller_collar", "ferrule", "crank")):
            part.set("material", "satin_metal")
        elif any(word in name for word in ("roller", "foot", "clamp", "pad")):
            part.set("material", "soft_plastic")
        if name == "used_abrasive":
            part.attrib.update(material="old_grit", rgba="1 1 1 1")


def decorate(tree, world, task):
    look(tree, world)
    functions = {"Printer": printer_hardware, "Olive": olive_hardware,
                 "Sander": sander_hardware, "Bobbin": bobbin_hardware,
                 "Screen": screen_hardware, "Chain": chain_hardware}
    functions[task](world)


def visual_box(body, name, p, size, color, **attrs):
    return geom(body, name, "box", p, size, color, group=2, contype=0, conaffinity=0, mass=0, **attrs)


def visual_disc(body, name, p, r, h, color, **attrs):
    return geom(body, name, "cylinder", p, (r, h), color, group=2, contype=0, conaffinity=0, mass=0, **attrs)


def hide_exact(body, *names):
    for part in body.iter("geom"):
        if part.get("name") in names:
            part.set("group", "5")


def hide_prefix(body, *prefixes):
    for part in body.iter("geom"):
        if part.get("name", "").startswith(prefixes):
            part.set("group", "5")


def screw(body, name, x, y, z, radius=.003):
    visual_disc(body, name, (x, y, z), radius, .001, METAL, material="satin_metal")
    visual_box(body, name + "_slot", (x, y, z + .0011), (radius * .72, .00045, .00013), INK,
               euler=(0, 0, .73))


def printer_hardware(w):
    hide_exact(w, "label_roll", "white_label", "stock_path", "leading_label", "pinch_fold")
    hide_prefix(w, "wrinkled_backing")
    for y in (-.117, .117):
        visual_box(w, f"side_seam{y}", (.028, y, .2405), (.100, .008, .0018), DARK)
        visual_box(w, f"side_foot{y}", (.094, y, .204), (.022, .010, .004), DARK)
        screw(w, f"service{y}", (.077), y, .243)
    visual_box(w, "front_fascia", (.133, 0, .230), (.006, .121, .019), (.21,.25,.26,1))
    visual_box(w, "output_slot", (.140, 0, .248), (.002, .071, .004), DARK)
    for i in range(37):
        y = -.060 + i * .0033
        visual_box(w, f"tear_tooth{i}", (.144, y, .244), (.0015, .0013, .0007), METAL)
    visual_disc(w, "status_lamp", (.105, .108, .249), .003, .0015, (.18,.69,.31,1))
    visual_box(w, "guide_rail_l", (.050, -.068, .258), (.042,.003,.003), (.26,.30,.30,1))
    visual_box(w, "guide_rail_r", (.050,  .068, .258), (.042,.003,.003), (.26,.30,.30,1))


def olive_hardware(w):
    for name, (x,y) in (("fixture", (.023,.078)), ("tray", (.023,-.068))):
        for i,(dx,dy) in enumerate(((-.025,-.023),(.025,-.023),(-.025,.023),(.025,.023))):
            screw(w,f"{name}_mount{i}",x+dx,y+dy,.224,.004)
    for i in range(7):
        visual_disc(w, f"copper_age{i}", (.023,.078,.237+.004*i), .01055,.00024,
                    (.53+.025*i,.25+.012*i,.13+.008*i,1),material="satin_metal")
    visual_box(w, "extractor_rest", (.143,.078,.223), (.034,.030,.018), DARK)
    for y in (.055,.101):
        visual_box(w,f"extractor_rest_lip_{y}",(.143,y,.247),(.030,.006,.009),DARK)
        screw(w,f"extractor_rest_fastener_{y}",.168,y,.256,.003)
    extract = w.find(".//body[@name='extractor']")
    for i in range(8):
        visual_disc(extract, f"screw_threads{i}", (0,0,.022+.0021*i), .0085,.00052,
                    (.32,.36,.38,1), material="satin_metal")
    for i in range(28):
        t = i * math.tau / 28
        visual_box(extract, f"collar_knurl{i}", (.023*math.cos(t),.023*math.sin(t),.008),
                   (.0011,.0008,.010), (.65,.46,.22,1), euler=(0,0,t))


def sander_hardware(w):
    hide_prefix(w, "grit_line")
    hide_exact(w, "new_abrasive", "folded_paper_edge")
    for y in (-.084,.084):
        for x in (-.005,.098):
            screw(w, f"pad_{x}_{y}", x,y,.258)
    for i in range(12):
        visual_box(w, f"motor_rib{i}", (.109,-.074+.013*i,.228), (.002,.003,.009), DARK)
    visual_box(w, "sander_shell_seam", (.111,0,.220), (.0015,.099,.0014), DARK)
    visual_box(w, "clamp_spring_mount", (.089,.091,.261), (.011,.012,.006), DARK)
    visual_box(w, "clamp_tab_mount", (.084,-.047,.255), (.014,.023,.003), DARK)
    for y in (-.088,-.078,.080,.090):
        visual_box(w, f"side_grip{y}", (.109,y,.226), (.003,.002,.012), INK)


def bobbin_hardware(w):
    hide_prefix(w, "tangle")
    hide_exact(w, "trapped_tail", "folded_thread_bundle")
    visual_disc(w, "bobbin_dark_well", (.083,.099,.250), .025,.001, DARK)
    bobbin = w.find(".//body[@name='bobbin']")
    visual_disc(bobbin, "bobbin_eye", (0,0,.00952), .0062,.00014,DARK)
    for i in range(7):
        visual_disc(bobbin, f"thread_winding{i}", (0,0,-.0056+.00175*i),
                    .0151+(.00028 if i % 2 else 0), .00043, (.11,.30+.01*i,.54+.018*i,1))
    for i in range(8):
        t=i*math.tau/8
        visual_disc(bobbin, f"bobbin_eye_{i}", (.013*math.cos(t),.013*math.sin(t),.0095),
                    .0017,.00012,DARK)
    for x,y in ((.020,.061),(.120,.133),(.121,.051)):
        screw(w, f"needle_plate_{x}_{y}",x,y,.250,.0035)
    for i in range(7):
        visual_box(w,f"needle_mark{i}",(.047+.010*i,.053,.2492),(.00035,.004,.00018),INK)
    for i in range(4):
        visual_box(w,f"feed_dog{i}",(.045+.004*i,.082,.250),(.001,.007,.001),DARK)
    for i in range(5):
        visual_box(w, f"case_seam{i}", (-.018,-.135+.011*i,.320), (.002,.0018,.013),DARK)


def screen_hardware(w):
    hide_prefix(w, "mesh_weft", "mesh_warp", "rubber_segment")
    for y in (-.133,.133):
        for x in (-.108,.145):
            visual_box(w,f"miter_{x}_{y}",(x,y,.232),(.010,.010,.001),(.37,.43,.46,1))
            screw(w,f"corner_{x}_{y}",x,y,.233,.0025)
    for x in (-.116,-.104,.133,.162):
        visual_box(w,f"extrusion_{x}",(x,0,.221),(.0013,.125,.0012),(.41,.46,.48,1))
    visual_box(w,"spline_channel",(.147,0,.224),(.005,.126,.0014),DARK)
    roller = w.find(".//body[@name='roller']")
    for i in range(8):
        visual_box(roller, f"wood_handle_grain{i}",(-.044+.005*i,0,.0563),
                   (.0011,.010,.0003),(.60,.38,.21,1),euler=(0,0,.07*i))
    hub = w.find(".//body[@name='roller_wheel']")
    for x in (-.0075,.0075):
        visual_disc(hub,f"hub_flange{x}",(x,0,0),.012,.0009,METAL,
                    euler=(0,math.pi/2,0),material="satin_metal")
        for i in range(8):
            a = math.tau*i/8
            visual_disc(hub,f"hub_mark{x}_{i}",(x*1.16,.008*math.cos(a),.008*math.sin(a)),
                        .0016,.0002,DARK,euler=(0,math.pi/2,0))


def chain_hardware(w):
    hide_prefix(w, "return_link", "loose_link")
    crank = w.find(".//body[@name='crank']")
    rear = w.find(".//body[@name='rear_sprocket']")
    face = crank.find("geom[@name='chainring_disk']")
    face.set("group", "5")
    visual_disc(crank,"chainring_hub", (0,0,0), .018,.006,METAL,
                euler=(0,math.pi/2,0),material="satin_metal")
    for i in range(5):
        a = math.tau*i/5
        visual_box(crank,f"spider{i}",(0,.030*math.cos(a),.030*math.sin(a)),
                   (.004,.021,.006),METAL,euler=(a,0,0),material="satin_metal")
        visual_disc(crank,f"spider_bolt{i}",(.005,.039*math.cos(a),.039*math.sin(a)),
                    .0037,.001,DARK,euler=(0,math.pi/2,0))
    for i in range(52):
        a = math.tau*i/52
        visual_box(crank,f"ring_web{i}",(0,.048*math.cos(a),.048*math.sin(a)),
                   (.004,.0049,.0034),METAL,euler=(a,0,0),material="satin_metal")
    for i in range(16):
        a = math.tau*i/16
        visual_box(rear,f"rear_tooth{i}",(0,.033*math.cos(a),.033*math.sin(a)),
                   (.005,.0035,.0024),METAL,euler=(a,0,0),material="satin_metal")
    for i in range(9):
        visual_box(crank,f"pedal_tread{i}",(.026+.005*i,-.052,.010),
                   (.001,.012,.001),DARK)
    visual_disc(rear,"freehub_end",(.009,0,0),.013,.003,DARK,euler=(0,math.pi/2,0))


class Drawing:
    def __init__(self, trial, scene):
        self.s = trial
        self.scene = scene
        self.offset = np.array((trial.fixture.x_offset,0,0))

    def shape(self, kind, sizes, position, color, orientation=None):
        scene = self.scene
        slot = scene.geoms[scene.ngeom]
        matrix = np.eye(3) if orientation is None else orientation
        mj.mjv_initGeom(slot, kind, np.asarray(sizes, dtype=float),
                        np.asarray(position, dtype=float) + self.offset, matrix.ravel(),
                        np.asarray(color, dtype=np.float32))
        scene.ngeom += 1

    def line(self, a, b, r, color):
        a, b = np.asarray(a), np.asarray(b)
        if np.linalg.norm(a-b) < 1e-7:
            return
        self.shape(mj.mjtGeom.mjGEOM_CAPSULE, (r, r, r), a, color)
        mj.mjv_connector(self.scene.geoms[self.scene.ngeom-1],
                         mj.mjtGeom.mjGEOM_CAPSULE,r,a+self.offset,b+self.offset)

    def lathe(self, p, r, h, color, axis=1):
        matrix = np.eye(3)
        if axis == 1:
            matrix = np.array(((1,0,0),(0,0,1),(0,-1,0)))
        elif axis == 0:
            matrix = np.array(((0,0,1),(1,0,0),(0,1,0)))
        self.shape(mj.mjtGeom.mjGEOM_CYLINDER,(r,h,0),p,color,matrix)

    def strip(self, a,b, width, half_thickness, color):
        a, b = np.asarray(a),np.asarray(b)
        along = b-a
        half_length = np.linalg.norm(along)*.5
        along /= 2*half_length
        across = np.cross((0,0,1),along)
        across /= np.linalg.norm(across)
        up = np.cross(along,across)
        self.shape(mj.mjtGeom.mjGEOM_BOX,(half_length+.00015,width,half_thickness),
                   (a+b)*.5,color,np.column_stack((along,across,up)))

    def curve(self, points, radius, color):
        for a,b in zip(points,points[1:]):
            self.line(a,b,radius,color)


def render_materials(trial, scene):
    functions = {"Printer": print_media, "Sander": abrasive_sheet, "Bobbin": loose_thread,
                 "Screen": screen_mesh, "Chain": chain_loop}
    draw = functions.get(type(trial.fixture).__name__)
    if draw:
        draw(Drawing(trial, scene))


def print_media(d):
    pull = max(0.,d.s.joint("backing_pull"))
    lift = max(0.,d.s.joint("roller_lift"))
    feed = max(0.,d.s.joint("print_feed"))
    radius = math.sqrt(.037**2 - feed * .00016 / math.pi)
    d.lathe((-.014,0,.270),radius,.063,STOCK)
    for y in (-.0631,.0631):
        for i in range(8):
            d.lathe((-.014,y+i*math.copysign(.000024,y),.270),radius-i*.003,.00012,
                    STOCK if i % 2 else (.81,.755,.65,1))
        d.lathe((-.014,y*1.003,.270),.012,.00028,INK)
    phase = feed/.037
    for i in range(18):
        y = -.058+i*.0068
        a = np.array((-.014+radius*math.sin(phase),y,.270+radius*math.cos(phase)))
        d.line(a,a+(0,.004,0),.00047,(.72,.68,.59,1))
    path = ((-.014,0,.270+radius),(.013,0,.292),(.046,0,.261),(.083,0,.256),
            (.121,0,.252),(.140,0,.250))
    for a,b in zip(path,path[1:]):
        d.strip(a,b,.061,.00055,STOCK)
    if feed > .0005:
        # A fed label leaves the tear bar and curls down as its unsupported length grows.
        n = max(1,math.ceil(feed/.010))
        for i in range(n):
            def point(u):
                return (.140+u,0,.250-.022*(u/.083)**2)
            a,b=point(feed*i/n),point(feed*(i+1)/n)
            d.strip(a,b,.059,.00065,STOCK if i < n-1 else (.86,.82,.73,1))
        for i in range(12):
            x=.141+feed-.024+i*.00165
            if x<=.143:
                continue
            z=.251-.022*((x-.140)/.083)**2
            d.line((x,-.028,z),(x,-.008,z),(.00030 if i%3 else .00048),INK)
    stretch = min(1., pull/.070)
    for i in range(13):
        def fold(t):
            height = (.010-.005*stretch)*math.sin(t*5.7*math.pi)*(1-.4*t)
            return np.array((.083+pull+t*(.074+.008*stretch), -.063-.024*t,
                             .255+height+.007*t+.008*lift/.045*(1-t)*(1-stretch)))
        a,b=fold(i/13),fold((i+1)/13)
        d.strip(a,b,.027-.012*i/13,.00068,KRAFT if i%4<2 else (.77,.63,.43,1))
    d.strip((.151+pull,-.095,.263),(.171+pull,-.097,.269),
            .010,.0014,(.79,.67,.49,1))


def abrasive_sheet(d):
    takeup=max(0.,d.s.joint("sheet_tension"))
    slack=max(0.,1-takeup/.025)
    grip = d.s.data.site("left_tcp").xpos-d.offset
    end = min(.113,max(.080,.072+takeup))
    def edge(t,x):
        y = -.090+t*(end+.090)
        bow = slack * .017 * math.sin(math.pi*t)**2
        ripple=slack*.004*math.sin(3*math.pi*t)*math.cos(13*x)
        return (x,y,.258+bow+ripple)
    for i in range(32):
        a,b=edge(i/32,.047),edge((i+1)/32,.047)
        shade=.025*(.5+.5*math.sin(i*22.5))
        d.strip(a,b,.056,.0007,(.56+shade,.33+shade,.17+shade,1))
    for x in (-.009,.103):
        d.curve([edge(t,x) for t in np.linspace(0,1,33)],.00043,(.72,.49,.29,1))
    a=edge(.91,-.007)
    p=(-.026,.073+takeup,.273)
    if "abrasive" in d.s.fixture.grips:
        p=(p[0],min(max(grip[1],p[1]-.005),p[1]+.009),p[2])
    d.strip(a,p,.013,.001,(.70,.49,.29,1))


def loose_thread(d):
    pull = max(0.,d.s.joint("thread_pull"))
    lift = max(0.,d.s.joint("bobbin_lift"))
    u=min(1.,pull/.040)
    pinch=np.array((.126+pull,-.009,.262))
    blue=(.14,.37,.64,1)
    for k in range(3):
        points=[]
        for t in np.linspace(0,math.tau,35):
            a=t*(1+k*.12)
            points.append((pinch[0]-(.022+.005*k)*(.6+.4*(1-u))*(1-math.cos(a)),
                           pinch[1]+(.016+.004*k)*math.sin(a)+.010*math.sin(t/2),
                           .253+.006*math.sin(t*2.2+k)+.006*(1-math.cos(t))))
        d.curve(points,.00054+.00006*k,blue)
    start=np.array((.084+.069*u,.098*(1-u)+.003*u,.264+.20*lift*(1-u)))
    finish=pinch+(-.018,.006,0)
    points=[]
    for t in np.linspace(0,1,30):
        p=(1-t)*start+t*finish
        p[0]+=.012*math.sin(5*math.pi*t)*math.sin(math.pi*t)*(1-.67*u)
        p[2]+=(.012+.008*u)*math.sin(math.pi*t)
        points.append(p)
    d.curve(points,.00072,(.18,.42,.71,1))


def screen_mesh(d):
    depths = np.array([max(0.,d.s.joint(f"spline_depth_{i}")) for i in range(7)])
    tension = max(0.,d.s.joint("mesh_tension"))
    remaining = np.clip(1.-depths/.013,0,1)
    captured = getattr(d.s.fixture,"captured",())
    retained = len(captured)/7
    def fabric(x,y):
        q=math.exp(-((y+.014)/.080)**4)
        bias=min(1.,max(0.,(x+.040)/.188))
        slack=max(0.,1.-(tension/.022+retained))
        wrinkle=.012*q*bias*slack*math.sin(58*y+10*bias)
        shear=.013*q*bias*(1.-retained)*min(tension/.023,1.)
        return (x,y+shear,.226+wrinkle)
    mesh=(.17,.21,.23,1)
    sample_y = np.linspace(-.125,.125,9)
    sample_x = np.linspace(-.106,.146,10)
    for x in np.linspace(-.106,.146,34):
        d.curve([fabric(x,y) for y in sample_y],.00034,mesh)
    for y in np.linspace(-.125,.125,34):
        d.curve([fabric(x,y) for x in sample_x],.00032,mesh)
    nodes = np.r_[[-.124,-.100],[-.081+.016*i for i in range(7)],[.039,.067,.124]]
    lifts = np.r_[[0,0],remaining,[0,0,0]]
    previous=None
    for y in np.linspace(-.124,.124,75):
        loose=np.interp(y,nodes,lifts)
        p=(.147+.004*loose,y,.233+.014*loose)
        if previous is not None:
            d.line(previous,p,.0037,(.082,.088,.090,1))
            if round((y+.124)*300) % 2 == 0:
                d.line(previous+np.array((0,0,.0033)),np.array(p)+(0,0,.0033),
                       .00033,(.22,.23,.23,1))
        previous=np.array(p)


def chain_loop(d):
    align = d.s.joint("chain_align")
    shifted = .028+align
    depth = max(0.,d.s.joint("chain_depth"))
    engaged = d.s.fixture.engaged
    # Sample the two sprocket arcs and their tangent runs as an open belt in Y/Z.
    c1 = np.array((-.052,.269))
    c2 = np.array(( .129,.278))
    r1,r2=.057,.035
    theta=math.atan2(*(c2-c1)[::-1])
    delta=math.acos((r1-r2)/np.linalg.norm(c2-c1))
    t_top,t_bottom=theta+delta,theta-delta
    unit=lambda t: np.array((math.cos(t),math.sin(t)))
    radii=np.r_[np.linspace(t_top,t_bottom+math.tau,75)]
    front=[c1+r1*unit(a) for a in radii]
    rear=[c2+r2*unit(a) for a in np.linspace(t_bottom,t_top,60)]
    bottom=[(1-t)*front[-1]+t*rear[0]+(0,-.005*math.sin(math.pi*t)) for t in np.linspace(0,1,50)]
    top=[(1-t)*rear[-1]+t*front[0]+(0,.005*math.sin(math.pi*t)) for t in np.linspace(0,1,55)]
    def pose(p,u=0.):
        fall = .0 if engaged else .002*u
        return (.073+u*shifted,p[0],p[1]+u*(.010-depth)-fall)
    path=[pose(p, max(0.,1-i/32)) for i,p in enumerate(front)]
    path.extend(pose(p) for p in bottom[1:])
    path.extend(pose(p) for p in rear[1:])
    path.extend(pose(p, min(1.,t*1.9)) for t,p in zip(np.linspace(0,1,len(top)),top[1:]))
    path=np.array(path)
    distances=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    length=distances[-1]
    pitch=length/56
    progress=(-r1*(d.s.joint("crank_angle")-d.s.fixture.at_engagement)) if engaged else 0.
    for i in range(56):
        def point(dist):
            return np.array([np.interp(dist%length,distances,path[:,a]) for a in range(3)])
        a,b=point(i*pitch+progress),point((i+1)*pitch+progress)
        if np.linalg.norm(a-b) > .020:
            continue
        for sign in (-1,1):
            dx=np.array((sign*.0045,0,0))
            d.line(a+dx,b+dx,.00176,(.35,.39,.41,1) if i%2 else (.18,.21,.22,1))
        d.lathe(a,.0026,.0054,(.24,.27,.28,1),axis=0)
