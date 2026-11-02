import math

rho = 2710          # density of aluminium [kg/m^3]
b = 0.35            # width of the aluminium profile [m]
b_delta = 0.035     
h = 0.20            # height of the aluminium profile [m]
h_delta = 0.025
t = 0.006           # thickness of the aluminium profile [m]
N_links = 5         # number of links in the chain
Lreach = 15         # reach of the chain [m]
overlap = 0.75      # overlap of the links [m]
A_list = []
m_list = []
g = 9.81            # acceleration due to gravity [m/s^2]
m_hose = 20
m_plate = 10 + 1 + m_hose / 2       # mass of plate, camera and hose estimate

Ltot = Lreach + overlap * (N_links - 1)
s = Ltot / N_links

# Compute area and mass of the aluminium profile
for i in range(N_links):
    A = (b - i*b_delta) * (h - i*h_delta) - ((b - i*b_delta) - 2 * t) * ((h - i*h_delta) - 2 * t)  # cross-sectional area of the aluminium profile [m^2]
    A_list.append(A)
    m = rho * A * s
    m_list.append(m)


for item in m_list:
    print(item)

m_sum = sum(m_list)

# Utregning av tyngdepunkt
d = [Lreach - (i*(s - overlap) + s/2) for i in range(1, N_links)]
y_tp = sum(m*di for m, di in zip(m_list[1:], d)) / sum(m_list[1:])
print(f"y_tp: {y_tp}")

# Lift 3: seksjon 2-5 + halve slangen ved bommens midtpunkt
m3 = sum(m_list[1:]) + m_hose / 2
y3 = (sum(m_list[1:])*y_tp + m_hose/2*Lreach/2) / m3

# Moment ved roten
M0 = g * (m_list[0]*s/2 + m3*(Lreach - y3) + (m_plate + 18.4 + 3) * Lreach)         # 18.4 = UR5, 3 = dyse

# Regulatorkrav: oversving < 15 %, innsvingning < 2 s
zeta = -math.log(0.15) / math.sqrt(math.pi**2 + math.log(0.15)**2)
wn = 4 / (zeta * 2)

print(f"Lift 2: m = {m_list[0]:.1f} kg, r = [0 0 {s/2:.2f}]")
print(f"Lift 3: m = {m3:.1f} kg, r = [0 -{y3:.2f} 0]")
print(f"M0 = {M0/1e3:.1f} kNm, zeta >= {zeta:.2f}, wn >= {wn:.2f} rad/s")
