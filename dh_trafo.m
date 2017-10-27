function D_vi = dh_trafo(alpha, a, d, theta)
    % alpha(i-1), a(i-1), d(i), theta(i): DH-Parameter
    % D_vi ist homogene Transformationsmatrix von i -> i-1

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Rotation
    A = Ax(-alpha) * Az(-theta);
    % Translation
    b = Ax(-alpha) * -[a; 0; d];
    % DH-Matrix
    D_vi = [A b; 0 0 0 1];
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end
