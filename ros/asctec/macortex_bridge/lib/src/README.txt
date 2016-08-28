Linux version of Cortex SDK
===========================

Compile this example code using the commands
  cmake .
  make

You will get a static library 'libcortex_sdk.a' and a test binary 'clienttest'.
As Cortex streams multi-cast data on port 1001, you need to be root to receive multi-cast data, e.g. by running
  sudo ./clienttest

To use it with ros, copy the libcortex_sdk.a file in macortex_bridge/lib folder

Tested with Ubuntu 10.04
