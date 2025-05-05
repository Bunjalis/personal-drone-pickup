import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



#
def plot_thrust_stand_data():
    # Load the CSV data
    data = pd.read_csv('thrust_data_ramp.csv')
    data['Thrust'] = data['Thrust'] * 9.80665


    # Group by throttle and calculate the average thrust
    grouped_data = data.groupby('Throttle', as_index=False).mean()

    # Filter data up to throttle 0.8 for quadratic fitting
    filtered_data = grouped_data[grouped_data['Throttle'] <= 0.8]

    # Convert thrust from kilograms to newtons
    

    # Perform quadratic fitting
    coefficients = np.polyfit(filtered_data['Throttle'], filtered_data['Thrust'], 2)
    quadratic_fit = np.poly1d(coefficients)

    # Generate throttle values for the quadratic curve
    throttle_fit = np.linspace(0, 0.8, 100)
    thrust_fit = quadratic_fit(throttle_fit)

    # Generate throttle values for the additional equation
    additional_throttle_fit = np.linspace(0, 0.8, 100)
    additional_thrust_fit = (additional_throttle_fit * 6000)**2 * 1.326e-07

    # Plot thrust vs throttle
    plt.figure(figsize=(10, 5))
    plt.plot(grouped_data['Throttle'], grouped_data['Thrust'], marker='o', label='Thrust vs Throttle')
    plt.plot(throttle_fit, thrust_fit, color='red', label='Model Fit')
    plt.plot(additional_throttle_fit, additional_thrust_fit, color='blue', linestyle='--', label='(x*6000)^2 * 1.326e-07')
    plt.xlabel('Throttle Input %')
    plt.ylabel('Thrust(N)')
    plt.title('Thrust vs Throttle')
    plt.grid(True)
    plt.legend()
    plt.savefig('thrust_vs_throttle_with_fit.png')

    # Print the quadratic equation
    equation = f"Quadratic Equation: {coefficients[0]:.4f}x^2 + {coefficients[1]:.4f}x + {coefficients[2]:.4f}"
    print(equation)
    plt.text(0.1, 0.8 * max(grouped_data['Thrust']), equation, color='red', fontsize=10)

    # Plot thrust vs time
    plt.figure(figsize=(10, 5))
    plt.plot(data.index, data['Thrust'], label='Thrust vs Time')
    plt.xlabel('Time (Index)')
    plt.ylabel('Thrust')
    plt.title('Thrust vs Time')
    plt.grid(True)
    plt.legend()
    plt.savefig('thrust_vs_time.png')

    plt.show()

if __name__ == '__main__':
    plot_thrust_stand_data()