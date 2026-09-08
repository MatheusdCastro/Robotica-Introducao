import numpy as np
import matplotlib.pyplot as plt
import time

# Parâmetros dados
alpha = np.array([0.0, 0.0, 0.0])
a = np.array([0.0, 1.5, 0.9])
d = np.array([0.0, 0.0, 0.0])
teta  = np.array([0.0, 0.0, 60.0])

def dh_transform(theta, alpha, a, d): # Monta a matriz a partir dos parêmtros DH
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    T = np.array([
        [ ct,    -st,       0,    a       ],
        [ st*ca,  ct*ca,  -sa,   -sa*d    ],
        [ st*sa,  ct*sa,   ca,    ca*d    ],
        [ 0,      0,       0,     1       ]
    ], dtype=float)
    return T

# Animação
plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect('equal', 'box')
ax.grid(True)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Simulação do Robo RRR')
ax.set_xlim(-2.0, 2.5)
ax.set_ylim(-0.5, 2.5)

for ct1 in range(1, 4):
    teta[0] = ct1 * 15.0
    for ct2 in range(1, 11):
        teta[1] = ct2 * 2.5
        th = np.deg2rad(teta)   # vetor dos 3 ângulos em rad

        # Monta as matrizes T1, T2, T3 
        T = [None]*3
        for k in range(3):
            T[k] = dh_transform(th[k], alpha[k], a[k], d[k])

        # Transformação total da base até o efetuador
        TT = T[0] @ T[1] @ T[2]
        T10 = T[0]
        T21 = T[1]
        T32 = T[2]
        T20 = T10 @ T21
        T30 = T20 @ T32

        # Posições dos elos (Px, Py, Pz)
        Px_2 = T20[0,3]
        Py_2 = T20[1,3]
        Px_3 = T30[0,3]   # Link imaginário entre a base e a ferramenta
        Py_3 = T30[1,3]
        L1_x = np.linspace(0.0, Px_2, 2)
        L1_y = np.linspace(0.0, Py_2, 2)
        L2_x = np.linspace(Px_2, Px_3, 2)
        L2_y = np.linspace(Py_2, Py_3, 2)
        ee_x = np.linspace(0.0, TT[0,3], 2) # link imaginário da base p end_effector
        ee_y = np.linspace(0.0, TT[1,3], 2)
        ax.cla()              
        ax.set_aspect('equal', 'box')
        ax.grid(True)
        ax.set_xlim(-2.0, 2.5)
        ax.set_ylim(-0.5, 2.5)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title(f'θ1={teta[0]:.1f}°, θ2={teta[1]:.1f}°, θ3={teta[2]:.1f}°')

        # Efetuador, L1 e L2
        ax.plot(ee_x, ee_y, marker='s', linestyle='-', label='end-effector')
        ax.plot(L1_x, L1_y, linestyle='-', linewidth=2, label='L1')
        ax.plot(L2_x, L2_y, linestyle='-', linewidth=2, label='L2')
        ax.plot(0, 0, 'ko')          
        ax.plot(Px_2, Py_2, 'ro') 
        ax.plot(Px_3, Py_3, 'bo')
        ax.legend(loc='upper left')

        plt.draw()
        plt.pause(0.001)
        time.sleep(0.15)

plt.ioff()
plt.show()