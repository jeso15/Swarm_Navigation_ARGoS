/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

#include <cstring>

#include <argos3/core/utility/logging/argos_log.h>
#include <random>
#include <chrono>
/****************************************/
/****************************************/


// Function to calculate and correct a byte based on parity bit
UInt8 fixByte(UInt8 byte) {
    UInt8 parity = 0;
    UInt8 temp = byte;
    for (int i = 0; i < 8; i++) {  // Check all bits including parity
        parity ^= (temp & 1);
        temp >>= 1;
    }

    // If parity is not even (i.e., error detected)
    if (parity != 0) {
        // Flip the last bit to try to correct the error
        byte ^= 1;
    }
    return byte; // Return the corrected byte without the parity bit
}

// Function to check and potentially correct the byte
bool checkAndCorrectParity(UInt8& byte, UInt8 magicNumber) {
    UInt8 fixedByte = fixByte(byte);
    //std::cout << "Fixed Byte: " << (int)fixedByte << ", Original Byte: " << (int)byte << std::endl;
    if (fixedByte == magicNumber) {
        byte = fixedByte; // Correct the reference byte if needed
        return true;
    }
    return false;
}


/*bool checkAndCorrectParity(UInt8& byte, UInt8 magicNumber) {
   UInt8 fixedByte = fixByte(byte);
   LOG << fixedByte << "@#" << byte << std::endl;
   if (fixedByte == magicNumber) {
      return true;
   }
   else false; 
   //  // Calculate the parity
   //  UInt8 parity = 0;
   //  UInt8 temp = byte;
   //  for (int i = 0; i < 8; i++) {  // Check all bits including parity
   //      parity ^= temp & 1;
   //      temp >>= 1;
   //  }
   //  // If parity is not 0, there is a potential error
   //  if (parity != 0) {
   //      for (int i = 0; i < 8; i++) {
   //          UInt8 flipped = byte ^ (1 << i);  // Flip each bit
   //          UInt8 testParity = 0;
   //          temp = flipped;
   //          for (int j = 0; j < 8; j++) {  // Recalculate parity for flipped byte
   //              testParity ^= temp & 1;
   //              temp >>= 1;
   //          }
   //          if (testParity == 0) {  // Check if the parity is now correct
   //              if ((flipped & ~1) == (magicNumber << 1)) {  // Verify it matches the expected magic number shifted left (to account for parity bit)
   //                  byte = flipped;  // Correct the byte
   //                  return true;  // Indicate error was corrected
   //              }
   //          }
   //      }
   //      return false;  // No correction found that matches the magic number
   //  }
   //  // No error detected, check if the byte matches the expected magic number
   //  return ((byte & ~1) == (magicNumber << 1));
}
*/

UInt8 addParityBit(UInt8 byte) {
    UInt8 parity = 0;
    UInt8 temp = byte;
    while (temp) {
        parity ^= temp & 1; // XOR all bits
        temp >>= 1;
    }
    return (byte << 1) | (parity & 1); // Shift byte left and add parity bit at the lowest bit
}







CFootBotDiffusion::CFootBotDiffusion() :

   


   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)), 
   dist(0.0, 20.0) // Initialize mean and stddev for your normal distribution here
   {
    // Seed the generator with the current time
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
   }

   
/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   rab_send      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
   rab_get       = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
   ledRing       = GetActuator<CCI_LEDsActuator                >("leds");
   encoder       = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   GetNodeAttributeOrDefault(t_node, "role", robot_role, robot_role);
   GetNodeAttributeOrDefault(t_node, "comm_range", comm_range, comm_range);

   // Set unique colors
   if (robot_role == 1) {
      ledRing->SetAllColors(CColor(0, 255, 0, 255));
   } else if (robot_role == 2) {
      ledRing->SetAllColors(CColor(255, 0, 0, 255));
   }

   // Initialize Nav Table
   

   if (robot_role == 1) {
      navTable[0] = {0, 0};
   }

   bestNavHeading = 0;
   bestNavDist = 0;
   distanceStar = -1;
   stepnum = 0;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Update local distance estimates */
   CCI_DifferentialSteeringSensor::SReading encoder_reading = encoder->GetReading();
   Real distance_moved = (encoder_reading.CoveredDistanceLeftWheel + encoder_reading.CoveredDistanceRightWheel) / 2;
   Real radians_rotated = (-encoder_reading.CoveredDistanceLeftWheel + encoder_reading.CoveredDistanceRightWheel) / encoder_reading.WheelAxisLength;
   
   
   
   for (auto i = navTable.begin(); i != navTable.end(); ++i) {
      navTable[i->first].distance += distance_moved;
   }
   if (robot_role == 2) {
      bestNavDist -= distance_moved; 
      bestNavHeading -= radians_rotated;
   }

   /* Process recieved messages */
   CCI_RangeAndBearingSensor::TReadings readings = rab_get->GetReadings();
   for (auto i = readings.begin(); i != readings.end(); ++i) {
      CCI_RangeAndBearingSensor::SPacket reading = *i;

      if (reading.Range > comm_range) {
         // LOG << reading.Range << "\n";
         continue; // Artificially limit the range of communication by ignoring comms from beyond that range
      }

      CByteArray data = reading.Data;
      UInt8 magic = data.PopFront<UInt8>();
      //if (checkAndCorrectParity(magic,6) != true) continue;  //use this if you want it with parity/error corrector
      if (magic != 6) continue; // use this if you want it without parity/error corrector
      UInt8 target_id = data.PopFront<UInt8>();
      UInt32 reported_sequence_num = data.PopFront<UInt32>();
      UInt32 cursed = data.PopFront<UInt32>();
      float reported_distance = * ( float * ) & cursed;

      if (robot_role == 2) {
      // LOG << "Recieved id " << (int)target_id << " num " << reported_sequence_num << " dist " << reported_distance << "\n";
      }
      /* Update navigation tables is new information is better */
      float computed_distance = reading.Range + reported_distance;
      if (navTable.find(target_id) == navTable.end() || (computed_distance < navTable[target_id].distance && reported_sequence_num >= navTable[target_id].sequence_number) ) {
         navTable[target_id] = {
            reported_sequence_num,
            computed_distance
         };
         // LOG << navTable[target_id].sequence_number << "\n";
      }

      /* Update navigation behavior is new information is better */
      if (robot_role == 2) {
         if (distanceStar == -1 || (reported_distance < distanceStar && reported_sequence_num >= sequenceNumberStar)) {
            distanceStar = reported_distance;
            sequenceNumberStar = reported_sequence_num;
            bestNavDist = reading.Range;
            bestNavHeading = reading.HorizontalBearing.GetValue() - 0.02; // Offset to avoid colision
            LOG << bestNavDist << " @ " << bestNavHeading << "\n";
         }
      }

   }

   /* Send Messages */
   bool time_to_send_update = true;
   if (time_to_send_update) {
      if (robot_role == 1) { // Robot is the target
         int self_id = 0;
         navTable[self_id].sequence_number += 1;
      }

      const int message_size = 10;
      CByteArray message = CByteArray();
      for (auto i = navTable.begin(); i != navTable.end(); ++i) {
         UInt8 magic = 6;

      

         // Simulate a noisy environment with a certain probability to alter the magic number
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disturb(0.0, 1.0);
        if (disturb(gen) < 0.2) {  // E.g., 20% chance to flip a bit in the magic number
            magic = addParityBit(magic);  // Introduce a bit flip error
         }

         message << magic;

         UInt8 id = (UInt8)(i->first);
         message << id;

         UInt32 sequence_num = navTable[i->first].sequence_number;
         message << sequence_num;
         
         float distance = navTable[i->first].distance;
         UInt32 dist_cursed = * ( UInt32 * ) &distance;
         message << dist_cursed;

         // LOG << "Sending id " << (int)id << " num " << sequence_num  << " dist " << distance << "\n";
         // LOG << message << "\n";
         
         
      }
      if (navTable.size() > 0)
         rab_send->SetData(message);

   }

   /* Most of the bots should wander randomly */
   if (robot_role == 0) {
      /* Get readings from proximity sensor */
      const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
      /* Sum them together */
      CVector2 cAccumulator;
      for(size_t i = 0; i < tProxReads.size(); ++i) {
         cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      }
      cAccumulator /= tProxReads.size();
      /* If the angle of the vector is small enough and the closest obstacle
      * is far enough, continue going straight, otherwise curve a little
      */
      CRadians cAngle = cAccumulator.Angle();
      if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
         cAccumulator.Length() < m_fDelta ) {
         /* Go straight */
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }
      else {
         /* Turn, depending on the sign of the angle */
         if(cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
         }
         else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
         }
      }
   } else if (robot_role == 2) {
      if (bestNavDist <= 0) {
         // Stop if you have arrived
      } else if (bestNavDist <= 15 && distanceStar == 0) {
         UInt32 current_time = CSimulator::GetInstance().GetSpace().GetSimulationClock();
         LOG << current_time << " Found!" << std::endl;
         CSimulator::GetInstance().Terminate();
      } else if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(CRadians(bestNavHeading)) ) {
         /* Go straight */
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      } else {
         // Turn towards best heading
         // LOG << "Best Heading" << bestNavHeading << "\n";
         if(bestNavHeading < 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
         }
         else {
            m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
         }
      }
   }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")