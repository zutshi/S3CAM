###### ###############################################################
# S3CAM: A Falsification Tool for Safety Properties of Hybrid Systems
###### ###############################################################

###################
# CONTENTS ###
###################

The archive containes
- Implementation: S3CAM.m
- graph search C code: ./src/graphNpaths

###################
# SETUP ###
###################
- Type 'make' to compile C files in the ./src/graphNpaths directory
- Type 'make' to compile C files in the ./src/graphNpaths/k_paths directory
- To compare against S-Taliro: Download and install S-Taliro v1.4 from https://sites.google.com/a/asu.edu/s-taliro/s-taliro/download
- Open Matlab 
        Enable Parallelization: Type 'matlabpool' in matlab to start worker threads

###################
# RUN ###
###################

- run_s3cam.m demos how to run S3CAM
         
Note: The path of the benchmark files need to be added into Matlab before they can be run. (Open and run the file in Matlab or use 'addpath()' )

###################
# Logs ###
###################

Each benchmark folder contains logs for all properties presented in the paper for both S-Taliro and Scatter-And-Simulate under respective folders.
