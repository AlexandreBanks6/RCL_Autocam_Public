import numpy as np 
import costComputation
import dVRKMotionSolver
import PyKDL
import time
import OptimizationDataLogger
import matplotlib.pyplot as plt
import csv
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
import os
## PURPOSE: To load recorded data from a ring and wire task and push it through a solver to quantify its capabilities


class solverSimulator():

    def __init__(self, filename):

        #create cost function
        self.cost = costComputation.autocamCost(kinematicsModel="Python")

        #initiliaze solver with the null conditions 
        self.cost.initializeConditions(
            q_des = np.zeros(6), 
            T_des = PyKDL.Frame(), 
            T_target = PyKDL.Frame(), 
            worldFrame= PyKDL.Frame(),
            ECM_T_PSM_RCM = PyKDL.Frame(), 
            psm3_T_cam= PyKDL.Frame(),
            distanceReg = 1.0, 
            orientationReg = 1.0, 
            similarityReg = 1.0, 
            positionReg= 1.0,
            desOrientationReg=1.0, #not in use
            desiredDistance = 0.01,
            offset=[0.0,0.0,0.0]

        )

        # self.motionSolver.prog.AddVisualizationCallback(self.cost.costCallback, self.motionSolver.q)

        self.results = {"id": [], "angleError":[], "positionError":[], "optimalCost":[], "success":[], "completionTime":[], "centroidViewAngle": [], "perpendicularAngle":[], "viewDistance": []}
        self.results_desiredPose = {"id": [], "angleError":[], "positionError":[], "optimalCost":[], "success":[], "completionTime":[], "centroidViewAngle": [], "perpendicularAngle":[], "viewDistance":[]}

        self.predictions = {"ECM_T_PSM":[]} #predicted pose of psm3 by solver
        self.desired = {"ECM_T_PSM":[]} #desired pose of psm3
        self.target = {"T_target": []} #contains target pose of ring (in coordinate system of psm3)

        self.experiment = [] #contains all of the experiment parameters and overall results

        #initialize dataloader
        self.loader = OptimizationDataLogger.OptimizationDataLogger()
        self.loader.initReading(filename)

    #PURPOSE: Steps the simulator by loading in a row, solving for joints, and recording the results 
    #REQUIRES: Number of samples in the dataset
    def stepSimulator(self, id, distanceRegArg= 1.0, orientationRegArg= 1.0, similarityRegArg= 1.0, positionRegArg= 1.0, desOrientationArg=1.0,verbose = False): 

        #READ IN ROW FROM DATA LOADER AND POPULATE SIMULATION
        i = 0
        #print(str(len(self.loader.ik_indices)) + " number of IK indices" )
        time.sleep(5)
        while(1):
            #print(i)
            if i > len(self.loader.ik_indices) - 1:
                break 
            success, system_time,q_des,T_des,T_target,worldFrame,ECM_T_PSM_RCM,psm3_T_cam,offset,IK_Triggered,ECM_T_PSM3 = self.loader.readDataRow(self.loader.ik_indices[i])
            if success == False:
                break
            
            if  IK_Triggered:
                #initiliaze solver with the row parameters
                self.cost.initializeConditions(
                    q_des = q_des, 
                    T_des = T_des, 
                    T_target =T_target, 
                    worldFrame= worldFrame,
                    ECM_T_PSM_RCM = ECM_T_PSM_RCM, 
                    psm3_T_cam= psm3_T_cam,
                    offset=offset,
                    distanceReg = distanceRegArg, 
                    orientationReg = orientationRegArg, 
                    similarityReg = similarityRegArg, 
                    positionReg= positionRegArg,
                    desOrientationReg=desOrientationArg, #not in use
                    desiredDistance = 0.11
                )

                #run solver
                start_time = time.time()
                success,q,optimal_cost = self.motionSolver.solve_joints(q_des)
                end_time = time.time()
                execution_time = end_time - start_time

                #compute error
                angleError, positionError = self.cost.computeArbitraryPoseError(q= q, T_1= ECM_T_PSM3, verbose = False)
                centroidViewAngle = np.rad2deg(np.arccos(-1 * (self.cost.centroidAngleError(self.cost.ECM_T_PSM(q)) - 1) ))
                perpendicularAngle = np.rad2deg(np.arccos(self.cost.perpendicularToFloorError(self.cost.ECM_T_PSM(q))))
                viewDistanceErr = self.cost.distanceError(self.cost.ECM_T_PSM(q))
                #save the results
                self.results["success"].append(success)
                self.results["id"].append(0)
                self.results["angleError"].append(angleError)
                self.results["positionError"].append(positionError)
                self.results["completionTime"].append(execution_time)
                self.results["optimalCost"].append(optimal_cost)
                self.results["centroidViewAngle"].append(centroidViewAngle)
                self.results["perpendicularAngle"].append(perpendicularAngle)
                self.results["viewDistance"].append(viewDistanceErr)
                

                angleError, positionError = self.cost.computeArbitraryPoseError(q= q, T_1= T_des, verbose= False)
                self.results_desiredPose["success"].append(success)
                self.results_desiredPose["id"].append(0)
                self.results_desiredPose["angleError"].append(angleError)
                self.results_desiredPose["positionError"].append(positionError)
                self.results_desiredPose["completionTime"].append(execution_time)
                self.results_desiredPose["optimalCost"].append(optimal_cost)
                self.results_desiredPose["centroidViewAngle"].append(centroidViewAngle)
                self.results_desiredPose["perpendicularAngle"].append(perpendicularAngle)
                self.results_desiredPose["viewDistance"].append(viewDistanceErr)

                self.predictions["ECM_T_PSM"].append(self.cost.ECM_T_PSM(q))
                self.desired["ECM_T_PSM"].append(T_des)

                #compute target in PSM3 coordinate system
                offsetMat = np.eye(4)
                offsetMat[0:3,3] = T_target[0:3,3] - offset
                offsetMat[0:3,0:3] = T_target[0:3,0:3]
                self.target["T_target"].append(offsetMat)


                if verbose:
                    print("Iter: " + str(id) + " Success: " + str(success) + " posErr: " + str(positionError) + " angErr: " + str(angleError) + " time: " + str(execution_time) + " centroidAng: " + str(centroidViewAngle) + " perpAngle: " +str(perpendicularAngle) + " viewDistance: " + str(viewDistanceErr))
            i += 1

    #PURPOSE: Evaluates mean performance of the solver after a number of trials are completed
    def evaluatePerformance(self):
        performance_mean = {}
        performance_std = {}
        for key in self.results:

            mean, std = self.computeStatistics(self.results[key])
            #if key == "centroidViewAngle":
             #   print(self.results[key])
              #  print(f"mean and std of key ={mean} and {std}")
            performance_mean[key+"_mean"] = mean
            performance_std[key+"_std"] = std
        
        self.experiment.append({"distanceReg": 0.0, "orientationReg": 0.0, "similarityReg": 0.0}) #broken feature
        print("Simulation Performance: ")
        print("mean: ", end="")
        print(performance_mean)
        print("std: ", end="")
        print(performance_std)


        performance_mean = {}
        performance_std = {}
        for key in self.results_desiredPose:
            mean, std = self.computeStatistics(self.results_desiredPose[key])
            performance_mean[key+"_mean"] = mean
            performance_std[key+"_std"] = std
        print()
        print("Simulation Performance desiredPose: ")
        print("mean: ", end="")
        print(performance_mean)
        print("std: ", end="")
        print(performance_std)     

        return performance_mean, performance_std




    def computeStatistics(self, data):
        mean_value = np.nanmean(data)
        std_dev =  np.nanstd(data, ddof=1)
        return mean_value, std_dev
    
    #PURPOSE: Runs an experiment over all of the data for a given set of regularization terms
    def run_experiment(self, distanceReg, orientationReg, similarityReg, positionReg,desOrientationReg, solverIterations, solverTolerance, max_solver_time, solverName,solver_algorithm):

        
                #initialize motion solver 
        self.motionSolver = dVRKMotionSolver.dVRKMotionSolver(
            cost_func = self.cost.computeCost,
            constraint_lb = self.cost.jointsLowerBounds,
            constraint_up = self.cost.jointsUpperBounds,
            n_joints = 6, 
            verbose = False,
            solver_iterations = solverIterations, 
            solver_tolerance= solverTolerance,
            max_solver_time= max_solver_time,
            solverName = solverName,
            solver_algorithm=solver_algorithm
            
        )
        
        
        #insert loop that iterates


        self.stepSimulator(id= 0, distanceRegArg=distanceReg, orientationRegArg=orientationReg, similarityRegArg=similarityReg, positionRegArg=positionReg,desOrientationArg=desOrientationReg)

        #after loop, evaluate the performance
        mean, std = self.evaluatePerformance()

        #save results
        folder, timestamp = self.saveExperiment(mean, std, distanceReg, orientationReg, similarityReg, positionReg,desOrientationReg, solverIterations, solverTolerance, max_solver_time, solverName=solverName)

        #plot results
        self.plot_dict_of_lists_subplots(folder, timestamp, self.results)
        self.plot_trajectories_with_rotation_3D(folder, timestamp, self.predictions["ECM_T_PSM"], self.desired["ECM_T_PSM"], self.target["T_target"])

 

    def plot_dict_of_lists(self, data_dict):
        """
        Takes a dictionary containing multiple lists and creates a separate plot for each key.

        Parameters:
        data_dict (dict): A dictionary where keys are strings and values are lists to be plotted.

        Example:
        data_dict = {
            'List 1': [1, 2, 3, 4],
            'List 2': [10, 20, 30, 40],
        }
        plot_dict_of_lists(data_dict)
        """
    
        for key, value in data_dict.items():
            plt.figure()  # Create a new figure for each plot
            plt.plot(value, marker='o',markersize=0.1 )  # Plot the list data
            plt.title(f'Plot for {key}')  # Set title to include the key
            plt.xlabel('Index')  # Label for x-axis
            plt.ylabel('Value')  # Label for y-axis
            plt.grid(True)  # Add grid for better readability
            plt.show()  # Display the plot

    def plot_dict_of_lists_subplots(self, folder, timestamp, data_dict):
        """
        Takes a dictionary containing multiple lists and creates a single figure
        with each list plotted in a separate subplot.

        Parameters:
        data_dict (dict): A dictionary where keys are strings and values are lists to be plotted.

        Example:
        data_dict = {
            'List 1': [1, 2, 3, 4],
            'List 2': [10, 20, 30, 40],
        }
        plot_dict_of_lists_subplots(data_dict)
        """
        
        num_plots = len(data_dict)
        fig, axes = plt.subplots(num_plots, 1, figsize=(8, num_plots * 3), sharex=True)
        
        if num_plots == 1:
            axes = [axes]  # Ensure axes is a list when there is only one subplot
        
        for ax, (key, value) in zip(axes, data_dict.items()):
            ax.plot(value, marker='o', markersize=0.01)
            ax.set_title(f'Plot for {key}')
            # ax.set_xlabel('Index')
            ax.set_ylabel('Value')
            ax.grid(True)

        plt.tight_layout()  # Adjust layout to prevent overlap
        plt.savefig(folder + "/" + "Metrics_" + timestamp + ".pdf", dpi=300, bbox_inches='tight')  # High resolution and adjust spacing

        plt.show()

    def saveExperiment(self, mean_dict, std_dict, distanceReg, orientationReg, similarityReg, positionReg,desOrientationReg, solverIterations, solverTolerance, max_solver_time, solverName):
        hyperparameters = {"distanceReg": distanceReg, "orientationReg": orientationReg, "similarityReg":similarityReg, "positionReg":positionReg, "desOrientationReg":desOrientationReg, "solverIterations":solverIterations, "solverTolerance":solverTolerance, "max_solver_time":max_solver_time, "solver":solverName}
        folderName, timestamp = self.create_folder()
        self.save_dicts_to_csv_with_timestamp(folderName,timestamp, mean_dict, std_dict, hyperparameters)
        return folderName, timestamp


    def create_folder(self):
        """
        Creates a folder with the specified name if it doesn't already exist.

        Parameters:
        folder_name (str): The name or path of the folder to create.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"optimizationResults/{timestamp}"
        try:
            os.makedirs(folder_name, exist_ok=True)  # Creates the folder, ignores if it exists
            print(f"Folder '{folder_name}' created or already exists.")
        except Exception as e:
            print(f"An error occurred while creating the folder: {e}")
        return folder_name, timestamp

    
    def save_dicts_to_csv_with_timestamp(self, folder, time,  dict1, dict2, dict3):
        """
        Saves three dictionaries to a CSV file named with the current timestamp,
        with the keys as the header row and the values as the second row.

        Parameters:
        dict1, dict2, dict3: Dictionaries to save in CSV format.
        """
        
        # Combine the dictionaries into one, assuming the keys are unique
        combined_dict = {**dict1, **dict2, **dict3}
        
        # Generate filename with the current timestamp
        filename = f"{folder}/output_{time}.csv"
        
        # Write to CSV
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Write the header (keys)
            writer.writerow(combined_dict.keys())
            
            # Write the values
            writer.writerow(combined_dict.values())

        print(f"Data saved to {filename}")

    def plot_trajectories_with_rotation_3D(self, folder, timestamp, traj1, traj2, traj3):
        """
        Plots the 3D trajectories of three lists of 4x4 homogeneous transformation matrices,
        including rotation visualization with arrows.

        Parameters:
        folder (str): Directory to save the plot.
        timestamp (str): Timestamp string to add to the filename.
        traj1 (list of np.ndarray): First list of 4x4 numpy arrays representing trajectory 1.
        traj2 (list of np.ndarray): Second list of 4x4 numpy arrays representing trajectory 2.
        traj3 (list of np.ndarray): Third list of 4x4 numpy arrays representing trajectory 3.
        """
        
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot each trajectory with rotation
        for traj, color, label in zip([traj1, traj2, traj3], ['y', 'm', 'c'], ['Predicted', 'Desired', 'Target']):
            points = np.array([mat[:3, 3] for mat in traj])  # Extract translation components
            ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', markersize=0.01, color=color, label=label)
            
            # Add rotation visualization
            for mat in traj:
                origin = mat[:3, 3]  # Translation part
                x_axis = mat[:3, 0] * 0.2  # Scale the x-axis vector for visualization
                y_axis = mat[:3, 1] * 0.2  # Scale the y-axis vector for visualization
                z_axis = mat[:3, 2] * 0.2  # Scale the z-axis vector for visualization
                
                # Plot arrows representing the orientation of the transformation matrix
                ax.quiver(*origin, *x_axis, color='r', length=0.001, normalize=True)  # X-axis in red
                ax.quiver(*origin, *y_axis, color='g', length=0.001, normalize=True)  # Y-axis in green
                ax.quiver(*origin, *z_axis, color='b', length=0.001, normalize=True)  # Z-axis in blue

        # Label the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectories with Rotation Visualization')
        ax.legend()

        # Save the plot
        plt.savefig(f"{folder}/3D_{timestamp}.pdf", dpi=300, bbox_inches='tight')
        plt.show()

   




    def generate_float_range(self, start, stop, num_steps):
        """
        Generates a list of floats between two given values.

        Parameters:
            start (float): The starting value of the range.
            stop (float): The ending value of the range.
            num_steps (int): The number of steps (elements) in the range.

        Returns:
            list: A list of floats from start to stop with num_steps elements.
        """
        return np.linspace(start, stop, num_steps).tolist()




""" 
GOALS: 
1. Determine baseline of performance of the solver throughout entire trajectory
    a. Mean error (angle and postion)
    b. Completion time
    c. Number of times triggered
2. Provide tools to determine optimal configuration of weights for optimization

"""

if __name__ == "__main__":
    filename = "Data_9"
    simulator = solverSimulator(filename)
    
    #Trying a bunch of different algorithms that run under the hood of the solver
    #solver_algorithms=["LD_MMA","LD_SLSQP"]
    # solver_algorithms = [ 
    # "GD_STOGO", "GD_STOGO_RAND", 
    # "LD_LBFGS", "LN_PRAXIS", "LD_VAR1", "LD_VAR2", "LD_TNEWTON", "LD_TNEWTON_RESTART", 
    # "LD_TNEWTON_PRECOND", "LD_TNEWTON_PRECOND_RESTART", "GN_CRS2_LM", "GN_MLSL", "GD_MLSL", 
    # "GN_MLSL_LDS", "GD_MLSL_LDS", "LD_MMA", "LN_COBYLA", "LN_NEWUOA", "LN_NEWUOA_BOUND", 
    # "LN_NELDERMEAD", "LN_SBPLX", "LN_AUGLAG", "LD_AUGLAG", "LN_AUGLAG_EQ", "LD_AUGLAG_EQ", 
    # "LN_BOBYQA", "GN_ISRES", "AUGLAG", "AUGLAG_EQ", "G_MLSL", "G_MLSL_LDS", 
    # "LD_SLSQP", "LD_CCSAQ", "GN_ESCH", "GN_AGS"]

    #Trying Different Weights
    # distanceRegs=[15,20,25]
    # orientationRegs=[15,20,25]
    # positionRegs=[15,20,25]
    # #for i in range(len(solver_algorithms)):
    #  #   print("Algorithm Name: "+str(solver_algorithms[i]))
    # for i in range(len(distanceRegs)):
    #     for j in range(len(orientationRegs)):
    #         for k in range(len(positionRegs)):
    #             distanceReg=distanceRegs[i]
    #             orientationReg=orientationRegs[j]
    #             positionReg=positionRegs[k]
    #             print("distanceReg: "+str(distanceReg))
    #             print("orientationReg: "+str(orientationReg))
    #             print("positionReg: "+str(positionReg))

    simulator.run_experiment(distanceReg=10.0, 
                            orientationReg=30.0, 
                            similarityReg=1.0, 
                            positionReg=25.0,
                            desOrientationReg=2000.0, 
                            solverIterations=50, 
                            max_solver_time=0.05, 
                            solverTolerance=1e-3,
                            solverName="NLOPT",
                            solver_algorithm="LD_SLSQP")    #Looks like the best algorithm

