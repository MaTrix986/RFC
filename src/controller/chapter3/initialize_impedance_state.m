function x = initialize_impedance_state(ref)
    % 状态 [pc(3); dpc(3); Qdc(4); v_o(3)]，Q=[eta;epsilon]。
    % 初始 c=d；v_o=Rc'*(omega_d-omega_c)=0。
    x = [ref.p; ref.dp; 1; zeros(6, 1)];
end
