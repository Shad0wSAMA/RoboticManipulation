acc = degps2accelTick(1287);
disp(acc);

function acc_tick = degps2accelTick(acc_degps2)
    ACC_REV_PER_MIN2_PER_UNIT = 214.577;  % Dynamixel spec
    DEG_PER_REV = 360.0;
    SEC_PER_MIN = 60.0;
    MAX_UNIT = 32767;   % Profile Acceleration register limit

    % deg/s^2 -> rev/min^2
    acc_rev_min2 = acc_degps2 / DEG_PER_REV * (SEC_PER_MIN^2);

    % rev/min^2 -> Dynamixel unit
    acc_unit = acc_rev_min2 / ACC_REV_PER_MIN2_PER_UNIT;

    acc_tick = int32(round(acc_unit));
    acc_tick = min(max(acc_tick, 0), MAX_UNIT);
end