# Recommended Data Format(one used in include .txt data file):

##### Each sensor measurement is formatted on it a line as:

gt: ground truth (actual values for the object's x and y positions and velocities), used for RMSE

L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy

R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy
