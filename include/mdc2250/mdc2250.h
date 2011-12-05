/*!
 * \file mdc2250/mdc2250.h
 * \author David Hodo <david.hodo@gmail.com>
 * \author William Woodall <wjwwood@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 William Woodall - David Hodo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for the Roboteq MDC2250 Motor 
 * Controller.
 * 
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * 
 */

#ifndef MDC2250_H
#define MDC2250_H

// Standard Library Headers
//#include <map>
//#include <string>
//#include <sstream>

// Serial interface library
#include <serial.h>

// Boost Headers (system or from vender/*)
#include "boost/function.hpp"

// Roboteq API Headers
#include "roboteq_api/ErrorCodes.h"
#include "roboteq_api/RoboteqDevice.h"

// Library Headers
#include "mdc2250_types.h"

namespace mdc2250 {

//! Structure to represent the current status of the controller
struct mdc2250_status {
    double M1_amps; //!< motor 1 current [amps]
    double M2_amps; //!< motor 2 current [amps]
    double B1_amps; //!< battery current 1 [amps]
    double B2_amps; //!< battery current 2 [amps]
    long E1_count; //!< encoder 1 counts - absolute
    long E2_count; //!< encoder 2 counts - absolute
    long E1_rel_count; //!< encoder 1 counts - relative
    long E2_rel_count; //!< encoder 2 counts - relative
    long M1_cmd; //!< motor 1 command
    long M2_cmd; //!< motor 2 command
    long E1_rpm; //!< encoder speed 1 [rpm]
    long E2_rpm; //!< encoder speed 2 [rpm]
    double time; //!< current time
    double driverVoltage; //!< driver voltage [V]
    double batVoltage; //!< main battery voltage [V]
    long fiveVVoltage; //!< 5V output voltage [mV]

    // fault flags
    bool overheat;
    bool overvoltage;
    bool undervoltage;
    bool shortCircuit;
    bool ESTOP;
    bool sepexFault;
    bool EEPROMFault;
    bool configFault;
};


/***** Function Typedefs *****/
typedef boost::function<void(const std::exception&)> ExceptionCallback;
typedef boost::function<void(mdc2250_status, RuntimeQuery::runtimeQuery)> RuntimeQueryCallback;


/*!
 * Represents an MDC2250 Device and provides and interface to it.
 */
class MDC2250 {
public:
  /*!
   * Constructs the MDC2250 object.
   */
  MDC2250();
  virtual ~MDC2250();

  /*!
   * Connects to the MDC2250 motor controller given a serial port.
   * 
   * \param port Defines which serial port to connect to in serial mode.
   * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
   * 
   * \throws ConnectionFailedException connection attempt failed.
   * \throws UnknownErrorCodeException unknown error code returned.
   */
  bool connect(std::string port);

  /*!
   * Disconnects from the MDC2250 motor controller given a serial port.
   */
  void disconnect();

  /*!
   * Sets up mdc2250 to output a list of queries at a set rate and saves the eeprom
   *
   * \param queries a string defining the queries to be sent - see page 98
   * in the Roboteq MDC2250 user manual for a list of possible queries.
   * Maximum length is 48 characters.
   * Example: "?A:?V:?T:" will request amps, volts, and temperature
   * \param period time in milliseconds between receiving telemetry
   */
  void setTelemetryString(std::string queries, long period);

  //! Starts reading continously from serial port
  void startContinuousReading();
  //! Stops reading continuously from serial port
  void stopContinuousReading();

  //! Sets the callback function for handling new runtime queries
  void setRuntimeQueryCallback(RuntimeQueryCallback callback);

  //! Clears the query history and stops sending queries
  void clearBufferHistory();
  /*!
   * Send the previous 16 queries at a set rate
   *
   * \param period time [ms] between queries
   */
  void sendQueryHistory(long period);

  void setAcceleration(int channel, int acceleration);
  void getAcceleration(int channel, int acceleration);

  void ESTOP();
  void ClearESTOP();
  void motorCmd(int channel, int command);
  void multiMotorCmd(int cmd1, int cmd2);
  void setPosition(int channel, int position);
  void setVelocity(int channel, int velocity);
  void setEncoderCounter(int channel, int value);

  void reset();
  void factoryReset();
  void saveEEPROM();

  void setWatchdogTimer(long ms);
  void setEncoderPPR(int channel, int ppr);
  long getEncoderPPR();
  void setMaxRPM(int channel, int mrpm);
  long getMaxRPM();

  bool sendCommand(std::string cmd);

private:
    serial::Serial my_port;  //!< serial port for communicating with the motor controller

    //! data callback for handling serial data
    void readDataCallback(std::string readData);
    void parsePacket(std::string packet);
    std::map<std::string, RuntimeQuery::runtimeQuery> parsingMap; //!< map used to parse query data
    bool waitForAck(); //!< block until a command acknowledgement is received
    bool ackReceived; //!< true if command acknowledgement has been received
    // TODO: add mutex and condition variable for ackReceived

    mdc2250_status curStatus;
    RuntimeQueryCallback queryCallback;
};

}
#endif
