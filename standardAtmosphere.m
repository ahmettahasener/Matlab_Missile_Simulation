function rho = standardAtmosphere(altitude)
    % standardAtmosphere Calculates air density at a given altitude.
    % Simplified International Standard Atmosphere Model (Troposphere)
    
    if altitude < 0
        altitude = 0; % Ensure altitude is not negative
    end

    % Sea level conditions
    rho0 = 1.225; % kg/m^3 (Air density at sea level)
    T0 = 288.15;  % K (Temperature at sea level)
    P0 = 101325;  % Pa (Pressure at sea level)
    L = 0.0065;   % K/m (Temperature lapse rate - Troposphere)
    R = 287.058;  % J/(kg*K) (Specific gas constant for dry air)
    g_const = 9.81; % m/s^2 (Gravitational constant)

    % Temperature variation with altitude (for Troposphere)
    T = T0 - L * altitude;

    % Pressure variation with altitude (for Troposphere)
    % If the temperature lapse rate is not zero:
    if L ~= 0
        P = P0 * (T / T0)^(g_const / (L * R));
    else % Isothermal atmosphere (L=0) - rarely used for very high altitudes
        P = P0 * exp(-g_const * altitude / (R * T0));
    end
    
    % Density calculation using the ideal gas law
    rho = P / (R * T);
    
    % Apply a simple upper limit (to prevent negative densities)
    if rho < 0
        rho = 0;
    end
end