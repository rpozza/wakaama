#ifndef FASTANALOGIN_H
#define FASTANALOGIN_H

/*
 * Includes
 */
#include "mbed.h"
#include "pinmap.h"

#if !defined TARGET_LPC1768 && !defined TARGET_KLXX && !defined TARGET_LPC408X && !defined TARGET_LPC11UXX && !defined TARGET_K20D5M && !defined TARGET_LPC11XX
    #error "Target not supported"
#endif

 /** A class similar to AnalogIn, only faster, for LPC1768, LPC408X and KLxx
 *
 * AnalogIn does a single conversion when you read a value (actually several conversions and it takes the median of that).
 * This library runns the ADC conversion automatically in the background.
 * When read is called, it immediatly returns the last sampled value.
 *
 * LPC1768 / LPC4088
 * Using more ADC pins in continuous mode will decrease the conversion rate (LPC1768:200kHz/LPC4088:400kHz).
 * If you need to sample one pin very fast and sometimes also need to do AD conversions on another pin,
 * you can disable the continuous conversion on that ADC channel and still read its value.
 *
 * KLXX
 * Multiple Fast instances can be declared of which only ONE can be continuous (all others must be non-continuous).
 *
 * When continuous conversion is disabled, a read will block until the conversion is complete
 * (much like the regular AnalogIn library does).
 * Each ADC channel can be enabled/disabled separately.
 *
 * IMPORTANT : It does not play nicely with regular AnalogIn objects, so either use this library or AnalogIn, not both at the same time!!
 *
 * Example for the KLxx processors:
 * @code
 * // Print messages when the AnalogIn is greater than 50%
 *
 * #include "mbed.h"
 *
 * FastAnalogIn temperature(PTC2); //Fast continuous sampling on PTC2
 * FastAnalogIn speed(PTB3, 0);    //Fast non-continuous sampling on PTB3
 *
 * int main() {
 *     while(1) {
 *         if(temperature > 0.5) {
 *             printf("Too hot! (%f) at speed %f", temperature.read(), speed.read());
 *         }
 *     }
 * }
 * @endcode
 * Example for the LPC1768 processor:
 * @code
 * // Print messages when the AnalogIn is greater than 50%
 *
 * #include "mbed.h"
 *
 * FastAnalogIn temperature(p20);
 *
 * int main() {
 *     while(1) {
 *         if(temperature > 0.5) {
 *             printf("Too hot! (%f)", temperature.read());
 *         }
 *     }
 * }
 * @endcode
*/
class FastAnalogIn {

public:
     /** Create a FastAnalogIn, connected to the specified pin
     *
     * @param pin AnalogIn pin to connect to
     * @param enabled Enable the ADC channel (default = true)
     */
    FastAnalogIn( PinName pin, bool enabled = true );
    
    ~FastAnalogIn( void )
    {
        disable();
    }
    
    /** Enable the ADC channel
    *
    * @param enabled Bool that is true for enable, false is equivalent to calling disable
    */
    void enable(bool enabled = true);
    
    /** Disable the ADC channel
    *
    * Disabling unused channels speeds up conversion in used channels. 
    * When disabled you can still call read, that will do a single conversion (actually two since the first one always returns 0 for unknown reason).
    * Then the function blocks until the value is read. This is handy when you sometimes needs a single conversion besides the automatic conversion
    */
    void disable( void );
    
    /** Returns the raw value
    *
    * @param return Unsigned integer with converted value
    */
    unsigned short read_u16( void );
    
    /** Returns the scaled value
    *
    * @param return Float with scaled converted value to 0.0-1.0
    */
    float read( void )
    {
        unsigned short value = read_u16();
        return (float)value * (1.0f/65535.0f);
    }
    
    /** An operator shorthand for read()
    */
    operator float() {
        return read();
    }

    
private:
    bool running;    
    char ADCnumber;
    volatile uint32_t *datareg;
};

#endif
