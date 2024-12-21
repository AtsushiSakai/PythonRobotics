import numpy as np
import matplotlib.pyplot as plt

def blending_function_1(t):
    return -t + 2*t**2 - t**3

def blending_function_2(t):
    return 2 - 5*t**2 + 3*t**3

def blending_function_3(t):
    return t + 4*t**2 - 3*t**3

def blending_function_4(t):
    return -t**2 + t**3

def plot_blending_functions():
    t = np.linspace(0, 1, 100)
    
    plt.plot(t, blending_function_1(t), label='b1')
    plt.plot(t, blending_function_2(t), label='b2')
    plt.plot(t, blending_function_3(t), label='b3')
    plt.plot(t, blending_function_4(t), label='b4')
    
    plt.title("Catmull-Rom Blending Functions")
    plt.xlabel("t")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.axhline(y=0, color='k', linestyle='--')
    plt.axvline(x=0, color='k', linestyle='--')
    plt.show()

if __name__ == "__main__":
    plot_blending_functions()