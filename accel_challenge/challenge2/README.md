- Open a terminal, run
    ```sh
    roscore
    ```
- Open 2nd terminal, run the following lines to pop out simulator:
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/run_simulator.sh
    ```
- Open 3rd terminal, run to start `crtk_interface`, which running controllers and rostopics
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/run_crtk_interface.sh
    ```

- Open 4rd terminal to run challenge scripts,
    run challenge#2 
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/init.sh
    python example/challenge2_traj.py 
    ```
    or

    run challenge#3 
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/init.sh
    python example/challenge3_traj.py 
    ```
