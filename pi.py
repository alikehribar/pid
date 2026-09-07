import math
import time
import random
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button
from matplotlib.animation import FuncAnimation


setpoint = 100
kp = 0.04175
ki = 0.0006
kd = 0.6486
dt =0.1
V =24
R_h = 10
t_env = 25
h=10
Cp= 900
u_max =1
tau = 10
storage0 =0
m= 0.05
m_heater = 0.002
Cp_heater = 450
plant_area = 0.021
K_env = (h * plant_area)
K_hp = 1.0
i_max = 0.77
tau_sensor = 3.0
sensor_noise = 0.05
tau_d = 5
dt_ctrl = 2.0
sp_rate = 0.2      
sp_tau = 20.0             
WINDOW = 600.0             
BUF = int((WINDOW / dt))  
FRAME_MS = 33              
MAX_CATCHUP = 50           
CTRL_EVERY = int(round((dt_ctrl / dt)))

def pid(Setpoint,pv,kp,ki,kd,integral,error_prev,dt,d_filt):
    error = (Setpoint - pv)
    derivative = ((error - error_prev) / dt )

    d_filt += (((derivative - d_filt) * dt) / tau_d)
    p_term = (kp * error)
    i_term = (ki * integral)
    d_term = (kd * d_filt)
    U_raw = ((p_term + i_term) + d_term)
    U = min(u_max, max(0.0, U_raw))
    if (ki > 0.0):
        stuck_high = ((U_raw > u_max) and (error > 0.0))
        stuck_low = ((U_raw < 0.0) and (error < 0.0))
        if (not (stuck_high or stuck_low)):
            integral += (error * dt)
        integral = min((i_max / ki), max(((-i_max) / ki), integral))
    return U, error, integral, i_term, d_term, d_filt

def heater(V, R, storage, dt, tau, T_heater):
   W = ((V * V) / R)
   storage += (W * dt)
   Wout = (storage / tau)
   storage -= (Wout * dt)
   Q = (Wout * dt)
   T_heater += (Q / (m_heater * Cp_heater))
   return T_heater, storage
    
def heat_transfer (T_heater,T_plant,dt,K_hp = 1.0):
   Q_dot = (T_heater-T_plant)*K_hp
   Q_d = Q_dot*dt
   return Q_d

def plant(Q_d, T_plant, T_env, m_plant, Cp_plant):
    Q_env = ((K_env * (T_plant - T_env)) * dt)
    Q_net = (Q_d - Q_env)
    T_plant += (Q_net / (m_plant * Cp_plant))
    return T_plant

def sensor(T_plant, T_sensor, dt, tau_sensor, sensor_noise):
    T_sensor += (((T_plant - T_sensor) * dt) / tau_sensor)
    reading = (T_sensor + random.gauss(0.0, sensor_noise))
    return T_sensor, reading

def reference(sp_target, sp_cmd, sp_filt, sp_rate, sp_tau, dt):
    step = (sp_rate * dt)
    sp_cmd += max((-step), min(step, (sp_target - sp_cmd)))
    sp_filt += (((sp_cmd - sp_filt) * dt) / sp_tau)
    return sp_cmd, sp_filt


t_buf = deque(maxlen=BUF)
plant_buf = deque(maxlen=BUF)
sp_buf = deque(maxlen=BUF)
u_buf = deque(maxlen=BUF)

gains = {"kp": kp, "ki": ki, "kd": kd}
target = {"sp": setpoint}       
extras = {"sp_rate": sp_rate, "speed": 1.0}
state = {}                   


def reset_state():
    """Put the whole rig back at ambient and empty the scrolling buffers."""
    random.seed(0)
    state.update(t=0.0, T_plant=t_env, T_heater=t_env, T_sensor=t_env,
                 storage=storage0, integral=0.0, error_prev=0.0, d_filt=0.0,
                 sp_cmd=t_env, sp_filt=t_env, step_n=0, U=0.0)
    for buf in (t_buf, plant_buf, sp_buf, u_buf):
        buf.clear()


def step_once():
    """Advance the rig by one dt. Same physics order as the old batch loop."""
    s = state
    s["T_sensor"], pv = sensor(s["T_plant"], s["T_sensor"], dt, tau_sensor,
                               sensor_noise)
    # The controller wakes up every dt_ctrl; between wake-ups u is simply held.
    if ((s["step_n"] % CTRL_EVERY) == 0):
        s["sp_cmd"], s["sp_filt"] = reference(target["sp"], s["sp_cmd"],
                                              s["sp_filt"], extras["sp_rate"],
                                              sp_tau, dt_ctrl)
        s["U"], error, s["integral"], _i, _d, s["d_filt"] = pid(
            s["sp_filt"], pv, gains["kp"], gains["ki"], gains["kd"],
            s["integral"], s["error_prev"], dt_ctrl, s["d_filt"])
        s["error_prev"] = error
    # u is now a steady DC level, not a duty cycle: 0 -> 0 V, 1 -> V.
    V_heater = (s["U"] * V)
    s["T_heater"], s["storage"] = heater(V_heater, R_h, s["storage"], dt, tau,
                                         s["T_heater"])
    Q_d = heat_transfer(s["T_heater"], s["T_plant"], dt, K_hp)
    s["T_heater"] -= (Q_d / (m_heater * Cp_heater))
    s["T_plant"] = plant(Q_d, s["T_plant"], t_env, m, Cp)
    s["t"] += dt
    s["step_n"] += 1
    for buf, value in ((t_buf, s["t"]), (plant_buf, s["T_plant"]),
                       (sp_buf, s["sp_filt"]), (u_buf, s["U"])):
        buf.append(value)


def style(ax, ylabel, ylim=None, loc="upper right"):
    """Apply the settings that both panels share."""
    ax.set_ylabel(ylabel, fontsize=8)
    if (ylim is not None):
        ax.set_ylim(*ylim)
    ax.grid(True)
    ax.legend(loc=loc, fontsize=7)


fig, (ax_temp, ax_u) = plt.subplots(2, 1, figsize=(10, 8), sharex=True,
                                    gridspec_kw={"height_ratios": [2, 1]})
fig.subplots_adjust(bottom=0.40, hspace=0.15, top=0.90)

line_sp, = ax_temp.plot([], [], linestyle="--", color="#8a8f98", linewidth=2,
                        label="Setpoint, ramped (C)")
line_p, = ax_temp.plot([], [], color="#3b7dd8", linewidth=2,
                       label="Plate temperature (C)")
style(ax_temp, "Temperature (C)", loc="lower right")

line_u, = ax_u.plot([], [], color="#d1730c", linewidth=1.5,
                    label="u, controller output (0-1)")
ax_u.set_xlabel("Time (s)")
style(ax_u, "u (unitless)", ((-0.05), 1.05))


def log_row(y, name, lo, hi, value):
    """A slider that moves in decades, plus a box holding the plain number."""
    sl = Slider(fig.add_axes([0.10, y, 0.58, 0.022]), name, lo, hi,
                valinit=math.log10(value))
    bx = TextBox(fig.add_axes([0.78, (y - 0.005), 0.14, 0.032]), "",
                 initial=f"{value:.4g}")
    sl.valtext.set_text(f"{value:.4g}")
    return sl, bx


def linear_row(y, name, lo, hi, value):
    """One linear slider plus its text box, stacked at height y."""
    sl = Slider(fig.add_axes([0.10, y, 0.58, 0.022]), name, lo, hi, valinit=value)
    bx = TextBox(fig.add_axes([0.78, (y - 0.005), 0.14, 0.032]), "",
                 initial=f"{value:.4g}")
    return sl, bx


s_speed, b_speed = linear_row(0.335, "speed x", 1.0, 100.0, 1.0)
s_kp, b_kp = log_row(0.29, "kp", -3.0, 1.0, kp)
s_ki, b_ki = log_row(0.245, "ki", -4.0, 0.0, ki)
# kd is linear so that it can be set exactly to 0 (D term off).
s_kd, b_kd = linear_row(0.20, "kd", 0.0, 2.0, kd)
# i_max caps how much of u the integral alone may command.
s_imax, b_imax = linear_row(0.155, "i_max", 0.2, 2.0, i_max)
# sp_rate is the climb slope: 5 C/s is steep enough to look like a step.
s_sprate, b_sprate = linear_row(0.11, "sp_rate", 0.01, 5.0, sp_rate)
b_sp = TextBox(fig.add_axes([0.18, 0.05, 0.12, 0.035]), "setpoint (C)  ",
               initial=str(setpoint))
btn_run = Button(fig.add_axes([0.45, 0.05, 0.10, 0.035]), "Pause")
btn_reset = Button(fig.add_axes([0.58, 0.05, 0.10, 0.035]), "Reset")


def refresh():
    """Push the buffers into the lines and slide the time axis to the right."""
    if (len(t_buf) == 0):
        return
    times = list(t_buf)
    line_p.set_data(times, list(plant_buf))
    line_sp.set_data(times, list(sp_buf))
    line_u.set_data(times, list(u_buf))
    right = max(state["t"], WINDOW)
    ax_u.set_xlim((right - WINDOW), right)     # sharex drags ax_temp along
    lo = min(min(plant_buf), min(sp_buf))
    hi = max(max(plant_buf), max(sp_buf))
    ax_temp.set_ylim((lo - 3.0), (hi + 5.0))
    ax_temp.set_title(f"t = {state['t']:.1f} s      "
                      f"setpoint {state['sp_filt']:.2f} C  ->  "
                      f"target {target['sp']:.4g} C  at {extras['sp_rate']:.3g} C/s\n"
                      f"plate {state['T_plant']:.2f} C     "
                      f"error {(state['sp_filt'] - state['T_plant']):+.2f} C     "
                      f"u = {u_buf[-1]:.3f}")


_wall = [time.monotonic()]


def frame(_n):
    """Run speed * real time worth of dt steps, so the plot flows at speed x."""
    now = time.monotonic()
    if running[0]:
        speed = extras["speed"]
        due = int((((now - _wall[0]) * speed) / dt))
        cap = int((MAX_CATCHUP * speed))
        if (due > cap):
            due = cap
            _wall[0] = now        # drop the backlog instead of jumping ahead
        else:
            _wall[0] += ((due * dt) / speed)
        for _ in range(due):
            step_once()
    else:
        _wall[0] = now            # paused wall time is not a backlog
    refresh()
    return ()


def setter(name):
    """Where a linear slider's value has to land."""
    if (name == "i_max"):
        def put(value):
            global i_max
            i_max = value
    elif (name == "sp_rate"):
        def put(value):
            extras["sp_rate"] = value
    elif (name == "speed"):
        def put(value):
            extras["speed"] = max(0.01, value)
    else:
        def put(value):
            gains[name] = value
    return put


def on_log_slider(name, box, slider):
    def handler(value):
        gain = (10.0 ** value)
        gains[name] = gain
        slider.valtext.set_text(f"{gain:.4g}")
        box.eventson = False
        box.set_val(f"{gain:.4g}")
        box.eventson = True
    return handler


def on_log_box(name, slider):
    def handler(text):
        try:
            value = float(text)
        except ValueError:
            return
        gains[name] = value
        if (value > 0.0):
            exponent = math.log10(value)
            if (slider.valmin <= exponent <= slider.valmax):
                slider.eventson = False
                slider.set_val(exponent)
                slider.valtext.set_text(f"{value:.4g}")
                slider.eventson = True
    return handler


def on_linear_slider(name, box):
    put = setter(name)
    def handler(value):
        put(value)
        box.eventson = False
        box.set_val(f"{value:.4g}")
        box.eventson = True
    return handler


def on_linear_box(name, slider):
    put = setter(name)
    def handler(text):
        try:
            value = float(text)
        except ValueError:
            return
        put(value)
        if (slider.valmin <= value <= slider.valmax):
            slider.eventson = False
            slider.set_val(value)
            slider.eventson = True
    return handler


def on_setpoint(text):
    """The new target only sets where the ramp is heading, not where it jumps."""
    try:
        target["sp"] = float(text)
    except ValueError:
        return


def on_run(_event):
    running[0] = (not running[0])
    btn_run.label.set_text("Pause" if running[0] else "Start")


def on_reset(_event):
    reset_state()
    _wall[0] = time.monotonic()


s_kp.on_changed(on_log_slider("kp", b_kp, s_kp))
s_ki.on_changed(on_log_slider("ki", b_ki, s_ki))
b_kp.on_submit(on_log_box("kp", s_kp))
b_ki.on_submit(on_log_box("ki", s_ki))
for _name, _slider, _box in (("kd", s_kd, b_kd), ("i_max", s_imax, b_imax),
                             ("sp_rate", s_sprate, b_sprate),
                             ("speed", s_speed, b_speed)):
    _slider.on_changed(on_linear_slider(_name, _box))
    _box.on_submit(on_linear_box(_name, _slider))
b_sp.on_submit(on_setpoint)
btn_run.on_clicked(on_run)
btn_reset.on_clicked(on_reset)

running = [True]   # frame() and on_run() read this; True means the sim starts moving
reset_state()
# The animation must stay in a module-level name or the collector kills it.
ani = FuncAnimation(fig, frame, interval=FRAME_MS, blit=False,
                    cache_frame_data=False)
plt.show()
