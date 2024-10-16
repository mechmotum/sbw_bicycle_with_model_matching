# Calculation of the model matching controller gains
This folder contains the script necessary to calculate the model matching controller gains.
The other files are either supporting this script, or are the results of this script.
Check the start of each file for a short discription of its function.

Main script:
`ss_meijaard_equations_simplifaction.py` - Calculates the numerical A and B matrix and the model matching gain matrices.

Output:
`[partial_]restriction_solution_[xxx]` - the solution for the chosen 'fixed' variables.

`bike_and_ref_variable_dependend_system_matrices_[xxx]` - Data files containing the speed dependend A and B matrix 

`model_matching_gains_measured_parameters` - Data file containing the speed dependend model matching gain matrices.

