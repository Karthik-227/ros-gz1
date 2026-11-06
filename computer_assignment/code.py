import math
import matplotlib.pyplot as plt

def main():
    # =================================================================
    # 1. SCHEME SELECTION
    # =================================================================
    print("--- GUIDANCE SCHEMES ---")
    print("1. Pure Pursuit (PP)")
    print("2. Deviated Pursuit (DP)")
    print("3. Line of Sight Guidance (LOSG)")
    print("4. Beam Rider (BR)")
    print("5. Proportional Navigation (PN)")
    print("6. True Proportional Navigation (TPN)")
    print("7. Pure Proportional Navigation (PPN)")
    
    sch = input("Select scheme (1-7): ")

    # =================================================================
    # 2. INITIAL SETTINGS (Short Variables)
    # =================================================================
    vm, vt = 400.0, 240.0         # Speeds [m/s]
    r0 = 7000.0                   # Initial range [m]
    th0 = math.radians(30.0)      # Init LOS angle
    al_t = math.radians(60.0)     # Target heading
    
    N, k = 5.0, 5.0               # Navigation & LOS gains
    dlt = math.radians(10.0)      # Deviation angle for DP
    at = 0.0                      # Target acceleration [m/s^2]
    
    r_cap = 10.0                  # Capture radius [m]
    dt, t_max = 0.01, 120.0       # Time step & limit
    
    xm, ym = 0.0, 0.0
    xt, yt = r0 * math.cos(th0), r0 * math.sin(th0)
    gm, gt = th0, al_t            # Headings

    t = 0.0
    hit = False
    r_min, pk_am = r0, 0.0
    
    # --- Data Lists for ALL 22 Variables ---
    t_l, r_l, lam_l, lamd_l = [], [], [], []
    xm_l, ym_l, xt_l, yt_l = [], [], [], []
    vxm_l, vym_l, vxt_l, vyt_l = [], [], [], []
    vrx_l, vry_l, vr_l, vth_l = [], [], [], []
    gm_l, gt_l, am_l, at_l = [], [], [], []
    v_ratio_l, psi_l = [], []     # New lists for velocity ratio and psi

    # =================================================================
    # 3. KINEMATIC LOOP
    # =================================================================
    print("\nSimulating...")
    while t < t_max:
        # Relative Position
        dx, dy = xt - xm, yt - ym
        r = math.sqrt(dx**2 + dy**2)
        r_min = min(r_min, r)

        if r <= r_cap:
            hit = True
            break

        # Velocities
        vxm, vym = vm * math.cos(gm), vm * math.sin(gm)
        vxt, vyt = vt * math.cos(gt), vt * math.sin(gt)

        # Relative Velocities (Target - Missile)
        vrx, vry = vxt - vxm, vyt - vym

        # Geometry: Radial (Vr) and Transverse (Vth) Velocities
        vr = (dx * vrx + dy * vry) / r     # Radial rel vel (dr/dt, negative of closing vel)
        vth = (dx * vry - dy * vrx) / r    # Transverse rel vel (r * dlam/dt)
        
        lam = math.atan2(dy, dx)           # LOS Angle
        lam_d = vth / r                    # LOS Rate (theta dot)
        
        # New Parameters
        v_ratio = vm / vt                  # Velocity of missile / velocity of Target
        psi = gt - lam                     # Target heading angle - LOS angle

        # Origin to target (for LOSG & BR)
        lt = math.atan2(yt, xt)
        lm = math.atan2(ym, xm) if (xm != 0 or ym != 0) else lt

        # ---- CONTROL LAWS ----
        am, turn_am = 0.0, 0.0

        if sch == '1':   # PP
            gm = lam
            am = turn_am = vm * lam_d
        elif sch == '2': # DP
            gm = lam + dlt
            am = turn_am = vm * lam_d
        elif sch == '3': # LOSG
            am = turn_am = k * (lt - lm)
        elif sch == '4': # BR
            am = turn_am = k * vm * (lt - lm)
        elif sch == '5': # PN
            am = turn_am = N * (-vr) * lam_d
        elif sch == '6': # TPN (Accel normal to LOS)
            am = N * (-vr) * lam_d
            turn_am = am * math.cos(gm - lam)
        elif sch == '7': # PPN (Accel normal to velocity)
            am = turn_am = N * vm * lam_d
        else:            
            am = turn_am = N * (-vr) * lam_d

        pk_am = max(pk_am, abs(am))

        # --- SAVE DATA ---
        t_l.append(t); r_l.append(r)
        lam_l.append(math.degrees(lam)); lamd_l.append(math.degrees(lam_d))
        xm_l.append(xm); ym_l.append(ym); xt_l.append(xt); yt_l.append(yt)
        vxm_l.append(vxm); vym_l.append(vym); vxt_l.append(vxt); vyt_l.append(vyt)
        vrx_l.append(vrx); vry_l.append(vry)
        vr_l.append(vr); vth_l.append(vth)
        gm_l.append(math.degrees(gm)); gt_l.append(math.degrees(gt))
        am_l.append(am / 9.81); at_l.append(at / 9.81)
        v_ratio_l.append(v_ratio); psi_l.append(math.degrees(psi))

        # Update Headings
        gt += (at / vt) * dt
        if sch not in ['1', '2']: 
            gm += (turn_am / vm) * dt
        
        # Update Positions
        xm += vxm * dt; ym += vym * dt
        xt += vxt * dt; yt += vyt * dt
        
        t += dt

    # =================================================================
    # 4. TERMINAL REPORT
    # =================================================================
    print("\n===========================================================")
    if hit:
        print("RESULT: INTERCEPTION")
        print(f" tf        = {t:.3f} s")
        print(f" Rf        = {r:.3f} m (Capture={r_cap}m)")
        print(f" peak|am|  = {pk_am:.3f} m/s^2 ({pk_am/9.81:.2f} g)")
    else:
        print("RESULT: MISS")
        print(f" t_miss    = {t_max:.3f} s")
        print(f" R_miss    = {r_min:.3f} m")
    print("===========================================================")

    # =================================================================
    # 5. PLOTTING DICTIONARY (22 Variables)
    # =================================================================
    p_vars = {
        '1': ('Time [s]', t_l), 
        '2': ('Mis X [m]', xm_l), '3': ('Mis Y [m]', ym_l),
        '4': ('Tgt X [m]', xt_l), '5': ('Tgt Y [m]', yt_l),
        '6': ('Range (r) [m]', r_l), 
        '7': ('LOS Ang (lam) [deg]', lam_l), '8': ('LOS Rate (lam_d) [deg/s]', lamd_l),
        '9': ('Mis Accel (am) [g]', am_l), '10': ('Tgt Accel (at) [g]', at_l),
        '11': ('Mis Head (gm) [deg]', gm_l), '12': ('Tgt Head (gt) [deg]', gt_l),
        '13': ('Rad Rel Vel (vr) [m/s]', vr_l), '14': ('Trans Rel Vel (vth) [m/s]', vth_l),
        '15': ('Mis Vel X (vxm) [m/s]', vxm_l), '16': ('Mis Vel Y (vym) [m/s]', vym_l),
        '17': ('Tgt Vel X (vxt) [m/s]', vxt_l), '18': ('Tgt Vel Y (vyt) [m/s]', vyt_l),
        '19': ('Rel Vel X (vrx) [m/s]', vrx_l), '20': ('Rel Vel Y (vry) [m/s]', vry_l),
        '21': ('Vel Ratio (vm/vt)', v_ratio_l), '22': ('Psi (gt-lam) [deg]', psi_l)
    }

    while True:
        print("\n--- PLOTTING MENU ---")
        num = input("Number of panels (1-6, 0=Exit): ")
        if num == '0': break
        try:
            n_p = int(num)
            if not (1 <= n_p <= 6): continue
        except ValueError: continue

        # Print options in columns for readability
        print("\n--- VARIABLES ---")
        keys = list(p_vars.keys())
        for i in range(0, len(keys), 2):
            k1 = keys[i]
            str1 = f"{k1}. {p_vars[k1][0]:<30}"
            if i+1 < len(keys):
                k2 = keys[i+1]
                print(f"{str1} | {k2}. {p_vars[k2][0]}")
            else:
                print(str1)

        p_chs = []
        for i in range(n_p):
            x_c = input(f"\nPanel {i+1} X-axis num: ")
            y_c = input(f"Panel {i+1} Y-axis num: ")
            p_chs.append((x_c, y_c))

        if n_p == 1:   r_p, c_p = 1, 1
        elif n_p == 2: r_p, c_p = 1, 2
        elif n_p == 3: r_p, c_p = 1, 3
        elif n_p == 4: r_p, c_p = 2, 2
        else:          r_p, c_p = 2, 3 

        plt.figure(figsize=(4.5 * c_p, 3.5 * r_p))

        for i, (x_c, y_c) in enumerate(p_chs):
            if x_c in p_vars and y_c in p_vars:
                plt.subplot(r_p, c_p, i + 1)
                
                xl, xlst = p_vars[x_c]
                yl, ylst = p_vars[y_c]
                
                plt.plot(xlst, ylst, 'b-', lw=1.5, label='Data')
                
                # Multi-line plot context for direct comparisons
                if x_c == '2' and y_c == '3': 
                    plt.plot(xt_l, yt_l, 'r--', label="Target")
                elif x_c == '1' and y_c == '2':
                    plt.plot(t_l, xt_l, 'r--', label="Tgt X")
                elif x_c == '1' and y_c == '3':
                    plt.plot(t_l, yt_l, 'r--', label="Tgt Y")

                plt.title(f"{yl} vs {xl}", fontsize=9)
                plt.xlabel(xl, fontsize=8); plt.ylabel(yl, fontsize=8)
                plt.legend(fontsize=7); plt.grid(True)

        plt.tight_layout()
        plt.show() 

if __name__ == '__main__':
    main()