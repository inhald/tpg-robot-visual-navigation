import random
import matplotlib.pyplot as plt
random.seed(12345)
def generate_and_plot_r_values():
    r = random.uniform(-1.0, 1.0)
    values = [r]

    for i in range(1000):
        # r = r * random.uniform(0.5, 1.5)
        # if random.uniform(0.0, 1.0) < 0.1:
        #     r = -r
        r = r + random.gauss(0, 0.1)
        values.append(r)

    # Plot
    plt.figure(figsize=(8, 4))
    plt.plot(values, marker='o')
    plt.title("Generated r Values")
    plt.xlabel("Step")
    plt.ylabel("r")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

generate_and_plot_r_values()
