function w_empty = weighing(engine_prop_unducted, engine_trans, V_e, S_b, S_f, num_passengers, w_engines, w_fuel, P_inst)
    if engine_trans == 1
        w_tr = 0.17;
    else
        w_tr = 0.275;
    end
    w_tr = unit_conversion(w_tr,'power');
    if engine_prop_unducted == 1
        w_prop = 0.175;
    else
        w_prop=0.5;
    end
    w_prop = unit_conversion(w_prop, 'power');
    V_gondola = 0.005 * V_e;
    w_envelope = (0.25 * S_b + 0.033*V_e)/0.825; %
    w_fins = S_f * 2.05;   
    w_rig = 0.0475 * w_fins;
    w_tail = w_fins + w_rig;
    w_lg = 0.008 * V_e; 
    w_con = 0.46 * (V_e)^0.67;
    w_ei = 0.037 * V_e;
    w_gondola = 10.75 * V_gondola;
    w_crew = 77 * num_passengers;
    w_misc = 0.011 * V_e;
    w_empty = w_envelope + w_tail + w_gondola + w_lg + w_engines + w_fuel + w_tr * P_inst + w_prop * P_inst + w_con + w_ei + w_misc + w_crew;
end