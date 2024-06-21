function DCMbn = dcm_update(DCMbn, euler)
% dcm_update: updates  body-to-nav DCM.
%
% INPUT
%   DCMbn,	3x3 body-to-nav DCM.
%   euler,	3x1 Euler angles [roll pitch yaw] (rad).
%
% OUTPUT
%   DCMbn,  3x3 updated DCM body-to-nav.


S = skewm(euler);
mag_e = norm(euler);

if mag_e < 1.E-8
    
    A = eye(3);
else
    
    % Rodrigues' formula
    % Titterton, A(k), Eq. 11.10, p. 312
    % Groves, Eq. 5.73
    % A = eye(3) + (sin(mag_e)/mag_e) * S + ((1-cos(mag_e))/(mag_e^2)) * S * S;
    
    % Exact expression. Groves, Eq. 5.69.
    A = expm(S);
end

% Titterton, Eq. 11.4, p. 311.
DCMbn = DCMbn * A;

% Brute-force orthogonalization, Groves, Eq 5.79
c1 = DCMbn(:,1);
c2 = DCMbn(:,2);
c3 = DCMbn(:,3);

c1 = c1 - 0.5 * (c1'*c2) * c2 - 0.5 * (c1'*c3) * c3 ;
c2 = c2 - 0.5 * (c1'*c2) * c1 - 0.5 * (c2'*c3) * c3 ;
c3 = c3 - 0.5 * (c1'*c3) * c1 - 0.5 * (c2'*c3) * c2 ;

% Brute-force normalization, Groves, Eq 5.80
c1 = c1 / sqrt(c1'*c1);
c2 = c2 / sqrt(c2'*c2);
c3 = c3 / sqrt(c3'*c3);

DCMbn = [c1 , c2 , c3 ];
