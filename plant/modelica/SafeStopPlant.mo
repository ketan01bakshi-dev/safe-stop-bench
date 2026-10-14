package SafeStopPlant "Safe-stop bench plant: actuators + vehicle. AI-assisted, synthetic, illustrative parameters (plant/spec/plant_spec.md)"
  constant Real pi = 3.141592653589793;

  model Actuators "Longitudinal path (dead time + first-order lag + limits), backup brake, steering (rate limit + lag): spec R1-R6"
    parameter Real g(unit = "m/s2") = 9.81 "gravity (a parameter, not a package constant, so unit checking can see it)";
    parameter Real dead_time(unit = "s") = 0.080 "R1";
    parameter Real tau(unit = "s") = 0.1 "R2 lag time constant (also the backup brake)";
    parameter Real max_accel(unit = "m/s2") = 2.5 "R3";
    parameter Real backup_dead_time(unit = "s") = 0.050 "R5";
    parameter Real steer_rate(unit = "deg/s") = 20.0 "R6";
    parameter Real steer_tau(unit = "s") = 0.05 "R6";
    parameter Real max_steer(unit = "deg") = 30.0 "R6";
    parameter Real slew_T(unit = "s") = 0.001 "numerical tracking time of the rate limiter (not a physical parameter)";
    input Real accel_cmd(unit = "m/s2");
    input Real steer_cmd(unit = "deg");
    input Real backup_decel(unit = "m/s2");
    input Real mu(unit = "1");
    input Real brake_factor(unit = "1");
    input Boolean steer_stuck;
    input Real steer_rate_factor(unit = "1");
    output Real a(unit = "m/s2") "delivered acceleration (after limits and backup brake)";
    output Real delta(unit = "deg", start = 0, fixed = true) "road-wheel angle";
    Real a_act(unit = "m/s2", start = 0, fixed = true) "lag state of the main longitudinal path";
    Real backup(unit = "m/s2", start = 0, fixed = true) "lag state of the backup brake";
    Real steer_target(unit = "deg", start = 0, fixed = true) "rate-limited steering target";
    Real a_dead(unit = "m/s2") "request after the dead time";
    Real a_in(unit = "m/s2") "request after the dead time and the brake-effectiveness fault";
    Real a_min(unit = "m/s2") "friction limit";
    Real target(unit = "deg");
    Real rate(unit = "deg/s");
  equation
    // a zero history before t = dead_time (spec R1): delay() alone would return accel_cmd(0) there, i.e. no dead time at start-up
    a_dead = if time < dead_time then 0 else delay(accel_cmd, dead_time);
    a_in = if a_dead < 0 then a_dead * brake_factor else a_dead;
    a_min = -mu * g;
    // R3: a limited integrator, the state does not wind up past a limit
    der(a_act) = if (a_act >= max_accel and a_in > a_act) or (a_act <= a_min and a_in < a_act) then 0 else (a_in - a_act) / tau;
    der(backup) = ((if time < backup_dead_time then 0 else delay(backup_decel, backup_dead_time)) - backup) / tau;
    target = max(-max_steer, min(max_steer, steer_cmd));
    rate = steer_rate * steer_rate_factor;
    der(steer_target) = if steer_stuck then 0 else max(-rate, min(rate, (target - steer_target) / slew_T));
    der(delta) = if steer_stuck then 0 else (steer_target - delta) / steer_tau;
    a = max(a_min, min(max_accel, a_act) - backup);
  end Actuators;

  model KinematicVehicle "Kinematic bicycle, v >= 0 (no rolling back): spec R7-R9"
    parameter Real g(unit = "m/s2") = 9.81 "gravity";
    parameter Real wheelbase(unit = "m") = 2.7;
    parameter Real v0(unit = "m/s") = 0 "initial speed";
    input Real a_cmd(unit = "m/s2") "acceleration delivered by the actuators";
    input Real delta(unit = "deg");
    input Real grade_pct(unit = "1") "road grade in percent, positive = uphill";
    input Real mu(unit = "1") "road friction (unused by the kinematic model; the actuators apply it)";
    output Real v(unit = "m/s", start = v0, fixed = true);
    output Real a_veh(unit = "m/s2");
    output Real yaw_rate(unit = "rad/s");
    output Real x(unit = "m", start = 0, fixed = true);
    output Real y(unit = "m", start = 0, fixed = true);
    output Real psi(unit = "rad", start = 0, fixed = true);
    Real a_total(unit = "m/s2");
  equation
    a_total = a_cmd - g * grade_pct / 100;
    a_veh = if v <= 0 and a_total <= 0 then 0 else a_total;
    der(v) = a_veh;
    // R7 standstill is EXACTLY zero: the bench and the controller test v == 0.0, and a fixed-step solver (the FMU) would
    // otherwise park a hair below zero. Found by the L2 bench regression on 14 Oct 2026.
    when v < 0 then
      reinit(v, 0);
    end when;
    yaw_rate = v / wheelbase * tan(delta * pi / 180);
    der(psi) = yaw_rate;
    der(x) = v * cos(psi);
    der(y) = v * sin(psi);
  end KinematicVehicle;

  model DynamicVehicle "Force-based longitudinal model (bench v2.5 semantics), kinematic bicycle laterally: spec R7-R10"
    parameter Real g(unit = "m/s2") = 9.81 "gravity";
    parameter Real wheelbase(unit = "m") = 2.7;
    parameter Real v0(unit = "m/s") = 0 "initial speed";
    parameter Real mass(unit = "kg") = 1500 "curb mass";
    parameter Real payload(unit = "kg") = 0;
    parameter Real rot_inertia_factor(unit = "1") = 0.04;
    parameter Real crr(unit = "1") = 0.015;
    parameter Real cd_a(unit = "m2") = 0.7;
    parameter Real rho(unit = "kg/m3") = 1.2;
    final parameter Real m(unit = "kg") = mass + payload;
    final parameter Real m_eff(unit = "kg") = m * (1 + rot_inertia_factor);
    input Real a_cmd(unit = "m/s2") "demand, calibrated on the curb mass";
    input Real delta(unit = "deg");
    input Real grade_pct(unit = "1");
    input Real mu(unit = "1");
    output Real v(unit = "m/s", start = v0, fixed = true);
    output Real a_veh(unit = "m/s2");
    output Real yaw_rate(unit = "rad/s");
    output Real x(unit = "m", start = 0, fixed = true);
    output Real y(unit = "m", start = 0, fixed = true);
    output Real psi(unit = "rad", start = 0, fixed = true);
    Real theta(unit = "rad");
    Real n(unit = "N") "normal force";
    Real f_x(unit = "N") "tyre force transmitted";
    Real f_res(unit = "N") "rolling + aero resistance";
    Real f_slope(unit = "N");
    Real a_total(unit = "m/s2");
  equation
    theta = atan(grade_pct / 100);
    n = m * g * cos(theta);
    f_x = max(-mu * n, min(mu * n, a_cmd * mass));
    f_slope = m * g * sin(theta);
    f_res = if v > 0 then crr * n + 0.5 * rho * cd_a * v * v else 0;
    a_total = (f_x - f_res - f_slope) / m_eff;
    a_veh = if v <= 0 and a_total <= 0 then 0 else a_total;
    der(v) = a_veh;
    // R7 standstill is EXACTLY zero: the bench and the controller test v == 0.0, and a fixed-step solver (the FMU) would
    // otherwise park a hair below zero. Found by the L2 bench regression on 14 Oct 2026.
    when v < 0 then
      reinit(v, 0);
    end when;
    yaw_rate = v / wheelbase * tan(delta * pi / 180);
    der(psi) = yaw_rate;
    der(x) = v * cos(psi);
    der(y) = v * sin(psi);
  end DynamicVehicle;

  partial model PlantBase "Inputs and outputs of the plant FMU (spec section 2)"
    parameter Real v0(unit = "m/s") = 0 "initial speed";
    input Real accel_cmd(unit = "m/s2", start = 0);
    input Real steer_cmd(unit = "deg", start = 0);
    input Real backup_decel(unit = "m/s2", start = 0);
    input Real mu(unit = "1", start = 0.8);
    input Real grade_pct(unit = "1", start = 0);
    input Real brake_factor(unit = "1", start = 1);
    input Boolean steer_stuck(start = false);
    input Real steer_rate_factor(unit = "1", start = 1);
    output Real a(unit = "m/s2");
    output Real delta(unit = "deg");
    output Real v(unit = "m/s");
    output Real a_veh(unit = "m/s2");
    output Real yaw_rate(unit = "rad/s");
    output Real x(unit = "m");
    output Real y(unit = "m");
    output Real psi(unit = "rad");
    Actuators act;
  equation
    act.accel_cmd = accel_cmd;
    act.steer_cmd = steer_cmd;
    act.backup_decel = backup_decel;
    act.mu = mu;
    act.brake_factor = brake_factor;
    act.steer_stuck = steer_stuck;
    act.steer_rate_factor = steer_rate_factor;
    a = act.a;
    delta = act.delta;
  end PlantBase;

  model Plant "Actuators + kinematic bicycle (the bench's default plant)"
    extends PlantBase;
    KinematicVehicle veh(v0 = v0);
  equation
    veh.a_cmd = act.a;
    veh.delta = act.delta;
    veh.grade_pct = grade_pct;
    veh.mu = mu;
    v = veh.v;
    a_veh = veh.a_veh;
    yaw_rate = veh.yaw_rate;
    x = veh.x;
    y = veh.y;
    psi = veh.psi;
  end Plant;

  model PlantDynamic "Actuators + force-based vehicle (run.py --plant dynamic)"
    extends PlantBase;
    parameter Real payload(unit = "kg") = 0;
    DynamicVehicle veh(v0 = v0, payload = payload);
  equation
    veh.a_cmd = act.a;
    veh.delta = act.delta;
    veh.grade_pct = grade_pct;
    veh.mu = mu;
    v = veh.v;
    a_veh = veh.a_veh;
    yaw_rate = veh.yaw_rate;
    x = veh.x;
    y = veh.y;
    psi = veh.psi;
  end PlantDynamic;
end SafeStopPlant;
