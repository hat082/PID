Set Display Environment Variable: After connecting to the remote machine, ensure that the DISPLAY environment variable is properly set. You can explicitly set the display using:

export DISPLAY=:0
Replace :0 with the correct display number if needed. You can find the display number by running the echo $DISPLAY command on the local machine.
