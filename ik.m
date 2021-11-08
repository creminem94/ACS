function [config1, config2] = ik()
    syms Px Pz a2 L1 L3;
    Py = sym('Py');
    dB = sym('dB');
    q3 = Pz - dB;
    q1_1 = Py - sqrt(a2^2-Px^2);
    q1_2 = Py + sqrt(a2^2-Px^2);
    q2_1 = atan2(Py-q1_1, Px);
    q2_2 = atan2(-Py+q1_2, Px);
    config1 = [q1_1-L1/2;q2_1;q3+L3/2];
    config2 = [q1_2-L1/2;q2_2;q3+L3/2];
end

