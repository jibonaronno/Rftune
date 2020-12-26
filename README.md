# STM32L496ZGTx C++ Code Base.
<img alt="NO IMAGE" src="STM32L496.png"><br>

Because of C++ incompatibility some extra code part is needed to apply. <br /> 
In main.cpp modify the function definition as :<br />
```c
extern "C" void SystemClock_Config(void);
```
<img alt="NO IMAGE" src="SystemClock_Config.png"><br>