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
    

    # Perform standard quadratic fitting with x*6000 and enforce y(0) = 0
    def constrained_quadratic_fit(x, y):
        # Fit a quadratic polynomial to y = a*(x*6000)^2 + b*(x*6000), enforcing c = 0
        scaled_x = x * 6000
        A = np.vstack([scaled_x**2, scaled_x]).T  # Design matrix for a and b
        coefficients, _, _, _ = np.linalg.lstsq(A, y, rcond=None)  # Solve for a and b
        return coefficients

    # Calculate the coefficients for the quadratic term
    coefficients = constrained_quadratic_fit(filtered_data['Throttle'], filtered_data['Thrust'])

    # Generate throttle values for the quadratic curve
    throttle_fit = np.linspace(0, 0.8, 100)
    scaled_throttle_fit = throttle_fit * 6000
    thrust_fit = coefficients[0] * scaled_throttle_fit**2 + coefficients[1] * scaled_throttle_fit

    # Generate throttle values for the additional equation
    additional_throttle_fit = np.linspace(0, 0.8, 100)
    additional_thrust_fit = (additional_throttle_fit * 6000)**2 * 1.326e-07

    # Plot thrust vs throttle
    plt.figure(figsize=(10, 5))
    plt.plot(grouped_data['Throttle'], grouped_data['Thrust'], marker='o', label='Thrust vs Throttle')
    plt.plot(throttle_fit, thrust_fit, color='red', label='Constrained Quadratic Fit (y(0)=0)')
    plt.plot(additional_throttle_fit, additional_thrust_fit, color='blue', linestyle='--', label='(x*6000)^2 * 1.326e-07')
    plt.xlabel('Throttle Input %')
    plt.ylabel('Thrust(N)')
    plt.title('Thrust vs Throttle')
    plt.grid(True)
    plt.legend()
    plt.savefig('thrust_vs_throttle_with_fit.png')

    # Print the quadratic equation
    equation = f"Quadratic Equation: {coefficients[0]:.4e} * (x*6000)^2 + {coefficients[1]:.4e} * (x*6000)"
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