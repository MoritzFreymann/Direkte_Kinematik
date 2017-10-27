function rob = berechne_dk_positionen_effizient(rob,aktuelles_gelenk,aktueller_zeitschritt)
    % Berechnung der Positions-Groessen der direkten Kinematik (Lage und Orientierung)
    % durch einen effizienten Algorithmus

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    % Berechnung fuer Koerper 1
    if ( aktuelles_gelenk == 1)
        
        erstes_gelenk = 2;  % erstes Gelenk, das in Schleife rekursiv berechnet wird ist das 2.
        
        i = 1;
        % Relativkinematik: Position und Orientierung relativ zum Vorgaenger
        % ------------------------------------------------------------------
        % Homogene Transformationsmatrix vom i-ten Koerper zum Vorgaenger
        % (Bitte nutzen Sie hierfuer auch die Datei dh_trafo.m)
        rob.kl(i).D_vi = dh_trafo( rob.kl(i).alpha, rob.kl(i).a, rob.kl(i).d, rob.q(i) );   % v_D_i
        
        % Drehmatrix vom Vorgaenger zum i-ten Koerper fuer Berechnung der Geschwindgkeiten
        rob.kl(i).A_iv = [   rob.kl(i).D_vi(1,1),    rob.kl(i).D_vi(1,2),    rob.kl(i).D_vi(1,3); 
                             rob.kl(i).D_vi(2,1),    rob.kl(i).D_vi(2,2),    rob.kl(i).D_vi(2,3);
                             rob.kl(i).D_vi(3,1),    rob.kl(i).D_vi(3,2),    rob.kl(i).D_vi(3,3)
                         ]';    % i_A_v = v_A_i^T
                     
        % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des
        % Vorgaengers (Bv) fuer erechnung der Geschwindkeiten
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];

        % Absolute Position und Orientierung
        % ----------------------------------
        % Homogene Transformationsmatrix vom i ins 0-System
        rob.kl(i).D_0i = rob.kl(i).D_vi; % 0_D_i = 0_D_v * v_D_i

        % Rotationsmatrix vom B0 ins Bi-KOS (aus homogener Transformationsmatrix)
        rob.kl(i).A_i0 =    [   rob.kl(i).D_0i(1,1),    rob.kl(i).D_0i(1,2),    rob.kl(i).D_0i(1,3); 
                                rob.kl(i).D_0i(2,1),    rob.kl(i).D_0i(2,2),    rob.kl(i).D_0i(2,3);
                                rob.kl(i).D_0i(3,1),    rob.kl(i).D_0i(3,2),    rob.kl(i).D_0i(3,3)
                            ]'; % = 0_A_i^T
        
        % Vektor vom Inertialsystem ins i-te KOS im B0-System (aus homogener Transformationsmatrix)
        rob.kl(i).B0_r_i = -[   rob.kl(i).D_0i(1,4);
                                rob.kl(i).D_0i(2,4);
                                rob.kl(i).D_0i(3,4)
                            ];  % 0_r_0,i = - 0_r_i,0
    else
        erstes_gelenk = aktuelles_gelenk;
    end
    % Berechnung fuer alle Koerper
    for i = erstes_gelenk:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % Relativkinematik: Position und Orientierung relativ zum Vorgaenger
        % ------------------------------------------------------------------
        % Homogene Transformationsmatrix vom i-ten Koerper zum Vorgaenger
        % (Bitte nutzen Sie hierfuer auch die Datei dh_trafo.m)
        rob.kl(i).D_vi = dh_trafo( rob.kl(i).alpha, rob.kl(i).a, rob.kl(i).d, rob.q(i) );   % v_D_i

        % Drehmatrix vom Vorgaenger zum i-ten Koerper fuer Berechnung der Geschwindgkeiten
        rob.kl(i).A_iv = [   rob.kl(i).D_vi(1,1),    rob.kl(i).D_vi(1,2),    rob.kl(i).D_vi(1,3); 
                             rob.kl(i).D_vi(2,1),    rob.kl(i).D_vi(2,2),    rob.kl(i).D_vi(2,3);
                             rob.kl(i).D_vi(3,1),    rob.kl(i).D_vi(3,2),    rob.kl(i).D_vi(3,3)
                         ]';    % i_A_v = v_A_i^T
                     
        % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des
        % Vorgaengers (Bv) fuer erechnung der Geschwindkeiten
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];
        
        % Absolute Position und Orientierung
        % ----------------------------------
        % Homogene Transformationsmatrix vom i ins 0-System
        rob.kl(i).D_0i = rob.kl(vor).D_0i * rob.kl(i).D_vi; % 0_D_i = 0_D_v * v_D_i

        % Vektor vom Inertialsystem ins i-te KOS im B0-System (aus homogener Transformationsmatrix)
        rob.kl(i).A_i0 =    [   rob.kl(i).D_0i(1,1),    rob.kl(i).D_0i(1,2),    rob.kl(i).D_0i(1,3); 
                                rob.kl(i).D_0i(2,1),    rob.kl(i).D_0i(2,2),    rob.kl(i).D_0i(2,3);
                                rob.kl(i).D_0i(3,1),    rob.kl(i).D_0i(3,2),    rob.kl(i).D_0i(3,3)
                            ]'; % = 0_A_i^T
                        
        % Rotationsmatrix vom B0 ins Bi-KOS (aus homogener Transformationsmatrix)
        rob.kl(i).B0_r_i =  -    [   rob.kl(i).D_0i(1,4);
                                     rob.kl(i).D_0i(2,4);
                                     rob.kl(i).D_0i(3,4)
                                 ];  % 0_r_0,i = - 0_r_i,0

    end
    % Position des Endeffektors im B0-System (aus homogener Transformationsmatrix)
    rob_w_4D = rob.kl(rob.N_Q).D_0i * [ rob.BN_r_N_tcp; 1 ];  % [w; 1] = 0_D_N_Q * [N_Q_r_N_Q,TCP; 1]
    rob.w = [
                rob_w_4D(1);
                rob_w_4D(2);
                rob_w_4D(3);
            ];
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end
