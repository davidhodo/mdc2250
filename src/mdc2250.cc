#include "mdc2250/mdc2250.h"
#include <sstream>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
using namespace mdc2250;
using namespace RuntimeQuery;
using namespace configitem;

/***** Inline Functions *****/

inline void defaultExceptionCallback(const std::exception &error) {
  std::cerr << "MDC2250 Unhandled Exception: " << error.what();
  std::cerr << std::endl;
  throw(error);
}

inline void defaultRuntimeQueryCallback(mdc2250_status status, runtimeQuery queryType) {
    //std::cout << "Parsed runtime query: " << queryType << std::endl;
}

inline void defaultConfigCallback(long value1, long value2, ConfigItem configType) {
    //std::cout << "Parsed config data: " << configType << std::endl;
}

// Reverse enum map for parsing query items
std::map<std::string, runtimeQuery> create_query_names() {
  std::map<std::string, runtimeQuery> m;
  m["A"] = _MOTAMPS;
  m["M"] = _MOTCMD;
  m["P"] = _MOTPWR;
  m["S"] = _ABSPEED;
  m["C"] = _ABCNTR;
  m["CB"] = _BLCNTR;
  m["VAR"] = _VAR;
  m["SR"] = _RELSPEED;
  m["CR"] = _RELCNTR;
  m["CBR"] = _BLRCNTR;
  m["BS"] = _BLSPEED;
  m["BSR"] = _BLRSPEED;
  m["A"] = _BATAMPS;
  m["V"] = _VOLTS;
  m["D"] = _DIGIN;
  m["DI"] = _DIN;
  m["AI"] = _ANAIN;
  m["PI"] = _PLSIN;
  m["T"] = _TEMP;
  m["F"] = _FEEDBK;
  m["FS"] = _STFLAG;
  m["FF"] = _FLTFLAG;
  m["DO"] = _DIGOUT;
  m["E"] = _LPERR;
  m["CIS"] = _CMDSER;
  m["CIA"] = _CMDANA;
  m["CIP"] = _CMDPLS;
  m["TM"] = _TIME;
  m["LK"] = _LOCKED;
  return m;
}

// Reverse enum map for parsing config items
std::map<std::string, ConfigItem> create_config_names() {
  std::map<std::string, ConfigItem> m;
  m["MRPM"] = _MXRPM;
  m["EPPR"] = _EPPR;
  return m;
}

/***** MDC2250 Class Functions *****/

MDC2250::MDC2250() {
    // Set default callback
    my_port.setReadCallback(boost::bind(&MDC2250::readDataCallback,this,_1));
    // create map for parsing queries
    queryMap=create_query_names();
    configMap=create_config_names();
    queryCallback=defaultRuntimeQueryCallback;
    configCallback=defaultConfigCallback;
}

MDC2250::~MDC2250() {
  this->disconnect();
}

bool MDC2250::connect(std::string port) {
    try {
        // configure and open serial port
        my_port.setPort(port);
        my_port.setBaudrate(115200);
        my_port.open();
        my_port.setTimeoutMilliseconds(50);

        // make sure port opened
        if (!my_port.isOpen()) {
            std::cout << "MDC2250: Serial port failed to open." << std::endl;
            return false;
        }

        clearBufferHistory();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        // poor man's flush
        my_port.read(10000);
        // make sure a mdc2250 is present on this port by asking for model number
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        my_port.write("\r?TRN\r"); // request model number

        // wait for return from serial port
        std::string result = my_port.read(30);
        // see if I got a valid response
        // response should look like 'TRN=RCB500:HDC2450'
        int pos1=result.find("TRN=");
        int pos2=result.find(":");
        if ((pos1>=0)&&(pos2>0)) {
            std::string unitID, modelID;
            unitID=result.substr(pos1+4,pos2-(pos1+4));
            modelID=result.substr(pos2+1,result.length()-pos2-2);
            std::cout << "Found Roboteq controller. Model: " << modelID << " Unit ID: " << unitID << std::endl;
            // compare model ID to mdc2250
            if (modelID.find("MDC2250")==std::string::npos) {
                std::cout << "Controller model is not supported." << std::endl;
                my_port.close();
                return false;
            }
        } else {
            std::cout << "Roboteq controller not found." << std::endl;
            my_port.close();
            return false;
        }

        // read firmware ID (FID)
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        my_port.write("\r?FID\r"); // request model number
        result = my_port.read(100);
        if (result.length()>10){
            std::cout << "Firmare ID: " << result.substr(10,result.length()-9) << std::endl;
        }

        my_port.setTimeoutMilliseconds(25);
    } catch (std::exception &e) {
        std::cout << "Failed to connect to MDC2250: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void MDC2250::disconnect() {
    my_port.close();
}

// TODO: add ability to give read_until char to continuous read
void MDC2250::setTelemetryString(std::string queries, long period) {
    std::stringstream cmd;
    cmd << "^TELS \"" << queries << ":# " << period << "\"\r";
    sendCommand(cmd.str());
}

void MDC2250::readDataCallback(std::string readData) {
    // split string on carriage returns '\r'
    std::vector< std::string > splitVec; // holds results
    boost::algorithm::split( splitVec, readData, boost::algorithm::is_any_of("\r"), boost::algorithm::token_compress_on );

    // loop through each message and parse it
    BOOST_FOREACH(std::string packet, splitVec)
    {
        if (packet.length()>1)
            parsePacket(packet);
    }
}

void MDC2250::parsePacket(std::string packet) {
    // possible data:
    // 1) command echo
    // 2) command acknowledgement (+ or -)
    // 3) query result
    // 4) ...
    //std::cout << "Parsing: " << packet << std::endl;
    runtimeQuery queryType;
    ConfigItem configType;
    // see if this is an echo of a command or query request
    try {
        int result = packet.find_first_of("!?%~^#");
        if (result!=std::string::npos) {
            //std::cout << "not processing" << std::endl;
            // echo of sent data - don't process
            return;
        }

        // check for command ack/nack
        if (packet.find_first_of("+-")!=std::string::npos) {
            // ack or nack - process
            if (packet.find("+"))
                std::cout << "Command acknowledged." << std::endl;
            if (packet.find("-"))
                std::cout << "Incorrect command received." << std::endl;
            return;
        }

        // split on equal sign
        std::vector< std::string > splitVec; // holds results
        boost::algorithm::split( splitVec, packet, boost::algorithm::is_any_of("=:"), boost::algorithm::token_compress_on );

        if (splitVec.size()<2) {
            std::cout << "Incorrectly formed query response: " << packet << std::endl;
            return;
        }


        double temp;
        bool queryFound=true;
        queryType=queryMap[splitVec[0]];
        // switch on the first
        switch (queryType) {
            case _MOTAMPS:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> temp;
                curStatus.M1_amps=temp/10.0;
                std::istringstream ( splitVec[2] ) >> temp;
                curStatus.M2_amps=temp/10.0;
                break;
            case _MOTCMD:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> curStatus.M1_cmd;
                std::istringstream ( splitVec[2] ) >> curStatus.M2_cmd;
                break;
            case _MOTPWR:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _ABSPEED:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> curStatus.E1_rpm;
                std::istringstream ( splitVec[2] ) >> curStatus.E2_rpm;
                break;
            case _ABCNTR:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> curStatus.E1_count;
                std::istringstream ( splitVec[2] ) >> curStatus.E2_count;
                break;
            case _BLCNTR:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _RELSPEED:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _RELCNTR:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> curStatus.E1_rel_count;
                std::istringstream ( splitVec[2] ) >> curStatus.E2_rel_count;
                break;
            case _BLRCNTR:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _BLSPEED:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _BLRSPEED:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _BATAMPS:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> temp;
                curStatus.B1_amps=temp/10.0;
                std::istringstream ( splitVec[2] ) >> temp;
                curStatus.B2_amps=temp/10.0;
                break;
            case _VOLTS:
                if (splitVec.size()<4) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> temp;
                curStatus.driverVoltage=temp/10.0;
                std::istringstream ( splitVec[2] ) >> temp;
                curStatus.batVoltage=temp/10.0;
                std::istringstream ( splitVec[3] ) >> curStatus.fiveVVoltage;
                break;
            case _DIGIN:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _DIN:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _ANAIN:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _PLSIN:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _TEMP:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _FEEDBK:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _STFLAG:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _FLTFLAG:
                if (splitVec.size()<2) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                int flag;
                std::istringstream ( splitVec[1] ) >> flag;
                // parse bits of fault flag
                curStatus.overheat=(flag&0x01)>0;
                curStatus.overvoltage=(flag&0x02)>0;
                curStatus.undervoltage=(flag&0x04)>0;
                curStatus.shortCircuit=(flag&0x08)>0;
                curStatus.ESTOP=(flag&0x10)>0;
                curStatus.sepexFault=(flag&0x20)>0;
                curStatus.EEPROMFault=(flag&0x40)>0;
                curStatus.configFault=(flag&0x80)>0;
                break;
            case _DIGOUT:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _LPERR:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _CMDSER:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _CMDANA:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _CMDPLS:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _TIME:
                std::cout << "Query not yet supported." << std::endl;
                break;
            case _LOCKED:
                std::cout << "Query not yet supported." << std::endl;
                break;
            default:
            std::cout << "Unrecognized query response: " << splitVec[0] << std::endl;
                queryFound=false;
                break;
        }

        if (queryFound) {
            // call query callback
            queryCallback(curStatus,queryType);
            return;
        }

        // if it was not a query see if it was a config item
        bool configFound=true;
        long value1=-1;
        long value2=-1;
        configType=configMap[splitVec[0]];
        switch (configType) {
            case _EPPR:
                if (splitVec.size()<3) {
                    std::cout << "Incorrectly formed query response: " << packet << std::endl;
                    return;
                }
                std::istringstream ( splitVec[1] ) >> value1;
                std::istringstream ( splitVec[2] ) >> value2;
                break;
            default:
                break;
        }

        if (configFound) {
            configCallback(value1, value2, configType);
            return;
        }

    } catch (std::exception &e) {
        std::cout << "Error parsing packet: " << e.what() << std::endl;
        return;
    }


}

bool MDC2250::waitForAck() {
    // TODO: implement this!
    // get lock on ackReceived
    // MUTEX HERE
    ackReceived=false;
    // wait for condition variable for a set time

    return false;
}

void MDC2250::startContinuousReading() {
    std::cout << "Starting continuous read." << std::endl;
    my_port.startContinuousRead(50);
}

void MDC2250::stopContinuousReading() {
    my_port.stopContinuousRead();
}

void MDC2250::setRuntimeQueryCallback(RuntimeQueryCallback callback) {
    queryCallback=callback;
}


/***** Command Methods *****/

void MDC2250::clearBufferHistory() {
    sendCommand("# C\r");
}

void MDC2250::sendQueryHistory(long period) {
    std::stringstream cmd;
    cmd << "# " << period << "\r";
    sendCommand(cmd.str());
}

inline void MDC2250::setAcceleration(int channel, int acceleration) {

}

inline void MDC2250::getAcceleration(int channel, int acceleration) {

}

inline void MDC2250::ESTOP() {
    sendCommand("!EX\r");
}

inline void MDC2250::ClearESTOP() {
    sendCommand("!MG\r");
}

void MDC2250::motorCmd(int channel, int command) {
    std::stringstream cmd;
    cmd << "!G " << channel << " " << command << "\r";
    sendCommand(cmd.str());
}

void MDC2250::multiMotorCmd(int cmd1, int cmd2) {
    std::stringstream cmd;
    cmd << "!M " << cmd1 << " " << cmd2 << "\r";
    sendCommand(cmd.str());
}

void MDC2250::setPosition(int channel, int position) {

}

void MDC2250::setVelocity(int channel, int velocity) {

}

void MDC2250::setEncoderCounter(int channel, int value) {

}

inline void MDC2250::reset() {
    sendCommand("%RESET 321654987\r");
}

inline void MDC2250::factoryReset() {
    sendCommand("%EERST 321654987\r");
}

inline void MDC2250::saveEEPROM() {
    sendCommand("%EESAV\r");
}

void MDC2250::setWatchdogTimer(long ms) {

}

void MDC2250::setEncoderPPR(int channel, int ppr=100) {
    // check range: 1 to 5000
    if ((ppr<1)||(ppr>5000)) {
        std::cout << "Invalid PPR value. Not set." << std::endl;
        return;
    }
    std::stringstream cmd;
    cmd << "^EPPR " << channel << " " << ppr << "\r";
    sendCommand(cmd.str());
}

long MDC2250::getEncoderPPR() {
    // see if continuous read is on
//    bool wasReading=my_port.isContinuouslyReading();
//    if (wasReading)
//        stopContinuousReading();

//    // request encoder PPR
//    sendCommand("\r~EPPR\r");
//    // wait for response


//    // restart continuous read if it was running
//    if (wasReading)
//        startContinuousReading();
    return 0;
}

void MDC2250::setMaxRPM(int channel, int mrpm=3000) {
    // check range 1 to 65000
    if ((mrpm<1)||(mrpm>65000)) {
        std::cout << "Invalid RPM value. Not set." << std::endl;
        return;
    }
    std::stringstream cmd;
    cmd << "^MRPM " << channel << " " << mrpm << "\r";
    sendCommand(cmd.str());
}

long MDC2250::getMaxRPM() {
    return 0;
}

bool MDC2250::sendCommand(std::string cmd) {
    if (!my_port.isOpen())
        return false;
    try {
        if (my_port.write(cmd)==cmd.length()) {
            // TODO: wait for ack??
            return true;
        } else
            return false;
    } catch (std::exception &e) {
        std::cout << "Failed to send command: " << e.what() << std::endl;
        return false;
    }
}

