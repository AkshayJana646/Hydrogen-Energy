from machine import ADC, Pin
import config

class VoltageDetector:
    
    adc = ADC(Pin(utils.VOLTAGE_PIN))
    adc.atten(ADC.ATTN_11DB)      # Measure up to ~3.3V
    adc.width(ADC.WIDTH_12BIT)    # 12-bit resolution (0–4095)
    VREF = 3.3  
    
    @staticmethod
    
    def read_voltage():
        """Read voltage from ADC and return in volts"""
        raw_value = VoltageDetector.adc.read()
        voltage = (raw_value / 4095) * VoltageDetector.VREF
        return voltage




