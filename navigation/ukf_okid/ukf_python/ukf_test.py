import numpy as np
def generate_delta_matrix(n: int) -> np.ndarray:
    if n < 1:
        raise ValueError("n must be a positive integer")

    delta = np.zeros((n, 2 * n))
    r_max = n // 2                  # floor(n/2)
    sq2 = np.sqrt(2.0)

    for k in range(1, 2 * n + 1):   # k = 1 … 2n
        for r in range(1, r_max + 1):
            row_cos = 2 * r - 2     # 0‑based index for γ_{k,2r‑1}
            row_sin = 2 * r - 1     # 0‑based index for γ_{k,2r}
            angle = (2 * r - 1) * k * np.pi / n
            delta[row_cos, k - 1] = sq2 * np.cos(angle)
            delta[row_sin, k - 1] = sq2 * np.sin(angle)

        if n % 2 == 1:              # extra entry when n is odd
            delta[n - 1, k - 1] = (-1) ** k

    return delta

a1 = np.array([1, 0, 1])
a2 = np.array([1, 1, 0])

a3 = np.outer(a1, a2)
print(a3)