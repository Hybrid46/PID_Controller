using System;
using UnityEngine;

[Serializable]
public class PIDController
{
    // Enumeration to define how to measure the derivative term
    public enum DerivativeMeasurementMode
    {
        Velocity,          // Measure based on change in velocity
        ErrorRateOfChange  // Measure based on change in error
    }

    // PID Coefficients (gains)
    public float proportionalGain;
    public float integralGain;
    public float derivativeGain;

    // Output limits
    public float outputMin = -1;
    public float outputMax = 1;

    // Integral windup limits
    public float integralSaturation;

    public DerivativeMeasurementMode derivativeMeasurementMode;

    // State variables for PID computation
    // Exposed for info display
    public float valueLast { get; private set; }
    public float errorLast { get; private set; }
    public float integrationStored { get; private set; }
    public float velocity { get; private set; }
    private bool derivativeInitialized;

    public PIDController(float proportionalGain, float integralGain, float derivativeGain, float integralSaturation, float outputMin = -1, float outputMax = 1)
    {
        this.proportionalGain = proportionalGain;
        this.integralGain = integralGain;
        this.derivativeGain = derivativeGain;
        this.integralSaturation = integralSaturation;
        this.outputMin = outputMin;
        this.outputMax = outputMax;
        Reset();
    }

    public void Reset()
    {
        derivativeInitialized = false;
        integrationStored = 0;
        errorLast = 0;
        valueLast = 0;
    }

    // General PID Update for linear values
    public float Update(float deltaTime, float currentValue, float targetValue)
    {
        ValidateDeltaTime(deltaTime);
        float error = targetValue - currentValue;

        float P = proportionalGain * error;

        integrationStored = Mathf.Clamp(integrationStored + (error * deltaTime), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        float D = ComputeDerivativeTerm(deltaTime, error, currentValue, false);

        float result = P + I + D;
        return Mathf.Clamp(result, outputMin, outputMax);
    }

    // Angle-specific PID Update for rotational values
    public float UpdateAngle(float deltaTime, float currentAngle, float targetAngle)
    {
        ValidateDeltaTime(deltaTime);
        float error = AngleDifference(targetAngle, currentAngle);

        float P = proportionalGain * error;

        integrationStored = Mathf.Clamp(integrationStored + (error * deltaTime), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        float D = ComputeDerivativeTerm(deltaTime, error, currentAngle, true);

        float result = P + I + D;
        return Mathf.Clamp(result, outputMin, outputMax);
    }

    // Computes the Derivative (D) term based on the chosen derivative measurement method
    private float ComputeDerivativeTerm(float deltaTime, float error, float currentValue, bool isAngle)
    {
        float errorRateOfChange;
        float valueRateOfChange;

        if (isAngle)
        {
            errorRateOfChange = AngleDifference(error, errorLast) / deltaTime;
            valueRateOfChange = AngleDifference(currentValue, valueLast) / deltaTime;
        }
        else
        {
            errorRateOfChange = (error - errorLast) / deltaTime;
            valueRateOfChange = (currentValue - valueLast) / deltaTime;
        }

        errorLast = error;
        valueLast = currentValue;
        velocity = valueRateOfChange;

        if (!derivativeInitialized)
        {
            derivativeInitialized = true;
            return 0; // Skip derivative term on the first iteration to avoid large spikes
        }

        float deriveMeasure = 0;
        if (derivativeMeasurementMode == DerivativeMeasurementMode.Velocity)
        {
            deriveMeasure = -valueRateOfChange;
        }
        else if (derivativeMeasurementMode == DerivativeMeasurementMode.ErrorRateOfChange)
        {
            deriveMeasure = errorRateOfChange;
        }

        return derivativeGain * deriveMeasure;
    }

    // Calculate the angular difference and remap it to [-180, 180] degrees
    private float AngleDifference(float a, float b)
    {
        return (a - b + 540) % 360 - 180;
    }

    private void ValidateDeltaTime(float deltaTime)
    {
        if (deltaTime <= 0f) throw new ArgumentOutOfRangeException(nameof(deltaTime), "Delta time must be positive and greater than zero.");
    }
}
