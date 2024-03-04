import numpy as np
import matplotlib.pyplot as plt

class MartianEnergySystem:
    def __init__(self, mass_salt_kg, num_wind_turbines, num_solar_panels):
        self.mass_salt = mass_salt_kg * 1000  # Convert kg to g for calculations
        self.num_wind_turbines = num_wind_turbines
        self.num_solar_panels = num_solar_panels
        self.temperature_initial = 20  # °C
        self.temperature_final = 850  # °C
        self.specific_heat_salt = 0.854  # J/g°C
        self.efficiency_electrolysis = 0.6  # Efficiency of electrolysis
        self.current = 10  # A (for electrolysis)
        self.time_seconds = 3600  # seconds (1 hour for electrolysis)
        self.electrochemical_equivalent_hydrogen = 0.01045  # mg/C (for hydrogen production)
        self.faradays_constant = 96485.33  # C/mol
        self.water_per_kg_salt_l = 1  # 1 liter of water per 1 kg of salt
        self.density_water = 1000  # g/L
        self.molar_mass_water = 18  # g/mol
        self.molar_mass_hydrogen = 2.01588  # g/mol for H2
        self.moles_hydrogen_per_mole_water = 2  # Stoichiometric production of H2 from water

    def calculate_energy_for_heating(self):
        delta_T = self.temperature_final - self.temperature_initial
        energy_joules = self.mass_salt * self.specific_heat_salt * delta_T
        return energy_joules

    def calculate_hydrogen_production(self):
        # Use stored instance attributes for the calculation
        volume_water_l = self.mass_salt / 1000 * self.water_per_kg_salt_l  # Volume of water in liters
        mass_water_g = volume_water_l * self.density_water  # Mass of water in grams
        
        # Calculate moles of water
        moles_water = mass_water_g / self.molar_mass_water
        
        # Calculate moles of hydrogen produced
        moles_hydrogen = moles_water * self.moles_hydrogen_per_mole_water
        
        # Calculate and return mass of hydrogen produced
        mass_hydrogen_g = moles_hydrogen * self.molar_mass_hydrogen
        
        return mass_hydrogen_g

    def calculate_daily_energy_needs(self):
        energy_heating_joules = self.calculate_energy_for_heating()
        total_energy_joules = energy_heating_joules / self.efficiency_electrolysis
        daily_energy_kwh = (total_energy_joules * 24) / (3600 * 1000)  # Convert J to kWh for 24 hours
        return daily_energy_kwh

    def calculate_renewable_energy_output(self):
        # Wind turbine and solar panel assumptions
        wind_turbine_output_mw = 2.5  # MW per turbine
        solar_panel_output_w = 300  # W per panel
        peak_sunlight_hours_per_day = 5  # hours
        # Total energy production
        wind_turbine_daily_kwh = self.num_wind_turbines * wind_turbine_output_mw * 1000 * 24
        solar_panel_daily_kwh = self.num_solar_panels * (solar_panel_output_w * peak_sunlight_hours_per_day) / 1000
        total_daily_kwh = wind_turbine_daily_kwh + solar_panel_daily_kwh
        return total_daily_kwh

    def simulate_martian_energy_system(self, mass_salt_kg_range):
        # Initialize lists to store results for plotting
        hydrogen_produced_list = []
        daily_energy_needs_list = []
        total_daily_energy_output_list = []
        
        for mass_salt_kg in mass_salt_kg_range:
            self.mass_salt = mass_salt_kg * 1000  # Update the mass of salt for each simulation
            hydrogen_produced = self.calculate_hydrogen_production()
            daily_energy_kwh = self.calculate_daily_energy_needs()
            total_daily_energy_output = self.calculate_renewable_energy_output()
            energy_surplus_deficit = total_daily_energy_output - daily_energy_kwh
            
            # Store results for plotting
            hydrogen_produced_list.append(hydrogen_produced)
            daily_energy_needs_list.append(daily_energy_kwh)
            total_daily_energy_output_list.append(total_daily_energy_output)
            
            print(f"Mass of Salt: {mass_salt_kg} kg, Hydrogen Produced: {hydrogen_produced} g, Daily Energy Needs: {daily_energy_kwh} kWh, Total Daily Energy Output: {total_daily_energy_output} kWh, Energy Surplus/Deficit: {energy_surplus_deficit} kWh")
        
        # Now plot the results
        plt.figure(figsize=(10, 6))
        plt.plot(mass_salt_kg_range, hydrogen_produced_list, marker='o', linestyle='-', color='b', label='Hydrogen Produced (g)')
        plt.title('Hydrogen Production vs. Mass of Salt')
        plt.xlabel('Mass of Salt (kg)')
        plt.ylabel('Hydrogen Produced (g)')
        plt.grid(True)
        plt.legend()
        plt.show()



# Inputs from the user
mass_salt_kg = 1000
num_wind_turbines = 10
num_solar_panels = 50
mass_salt_kg_range = np.linspace(1, 10, 10)  # Define a range of salt masses
martian_energy_system = MartianEnergySystem(1, 5, 20)  # Initial example parameters
martian_energy_system.simulate_martian_energy_system(mass_salt_kg_range)
