#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def survival_fraction(k, D):
    """Calculate the survival fraction based on UV exposure dose."""
    return np.exp(-k * D)

# Parameters
k = 0.0701  # UV rate constant for our example virus (generic value, might vary in reality)
D = np.linspace(0, 40, 100)  # UV exposure dose ranging from 0 to 20 J/m^2

# Calculate survival fraction
S = survival_fraction(k, D)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(D, np.log10(S)+1, '-r', label="Log reduction")
plt.plot(D,S, 'b',label="no log")
plt.title("Log Reduction of Virus Population with UV Exposure")
plt.xlabel("UV Exposure Dose (J/m^2)")
plt.ylabel("Log Reduction")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()