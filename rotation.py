import sympy as sp

# Define symbolic variables
phi, theta, psi = sp.symbols('phi theta psi')

# Define rotation matrices
R1 = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(phi), sp.sin(phi)],
    [0, -sp.sin(phi),  sp.cos(phi)]
])

R2 = sp.Matrix([
    [sp.cos(theta), 0, -sp.sin(theta)],
    [0, 1, 0],
    [sp.sin(theta), 0, sp.cos(theta)]
])

R3 = sp.Matrix([
    [sp.cos(psi), sp.sin(psi), 0],
    [-sp.sin(psi),  sp.cos(psi), 0],
    [0, 0, 1]
])

# Combine the rotations
R = R1 * R2 * R3

# Simplify the result
R = sp.simplify(R)

# Print the result
sp.pprint(R, use_unicode=True)
