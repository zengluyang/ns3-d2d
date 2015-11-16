#ifndef _SUMO_TCL_PARSER_H_
#define _SUMO_TCL_PARSER_H_

#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <cstring>

#include "basic-models/mobility/sumo-trace-map.h"
#include "basic-models/mobility/sumo-parser.h"
#include "math/geometry/point.h"
#include "simcore/simtime.h"

using namespace std;

// Constants definitions
#define  NS2_AT       "at"
#define  NS2_X_COORD  "X_"
#define  NS2_Y_COORD  "Y_"
#define  NS2_Z_COORD  "Z_"
#define  NS2_SETDEST  "setdest"
#define  NS2_SET      "set"
#define  NS2_NODEID   "$node_("
#define  NS2_NS_SCH   "$ns_"

class Vector3D
{
public:
  /**
   * \param [in] _x X coordinate of vector
   * \param [in] _y Y coordinate of vector
   * \param [in] _z Z coordinate of vector
   *
   * Create vector (_x, _y, _z)
   */
  Vector3D (double _x, double _y, double _z);
  /**
   * Create vector (0.0, 0.0, 0.0)
   */
  Vector3D ();
  /**
   * x coordinate of vector
   */
  double x;
  /**
   * y coordinate of vector
   */
  double y;
  /**
   * z coordinate of vector
   */
  double z;
  
  friend double CalculateDistance (const Vector3D &a, const Vector3D &b);
  friend std::ostream &operator << (std::ostream &os, const Vector3D &vector);
  friend std::istream &operator >> (std::istream &is, Vector3D &vector);
};

/**
 * \brief a 2d vector
 * \see attribute_Vector2D
 */
class Vector2D
{
public:
  /**
   * \param [in] _x X coordinate of vector
   * \param [in] _y Y coordinate of vector
   *
   * Create vector (_x, _y)
   */
  Vector2D (double _x, double _y);
  /**
   * Create vector vector (0.0, 0.0)
   */
  Vector2D ();
  /**
   * x coordinate of vector
   */
  double x;
  /**
   * y coordinate of vector
   */
  double y;

  friend double CalculateDistance (const Vector2D &a, const Vector2D &b);
  friend std::ostream &operator << (std::ostream &os, const Vector2D &vector);
  friend std::istream &operator >> (std::istream &is, Vector2D &vector);
};

/**
 * \param [in] a One point
 * \param [in] b Another point
 * \returns The cartesian distance between a and b.
 */
double CalculateDistance (const Vector3D &a, const Vector3D &b);
/**
 * \param [in] a One point
 * \param [in] b Another point
 * \returns The cartesian distance between a and b.
 */
double CalculateDistance (const Vector2D &a, const Vector2D &b);

/**
 * Output streamer.
 *
 * Vectors are written as "x:y:z".
 *
 * \param [in,out] os The stream.
 * \param [in] vector The vector to stream
 * \return The stream.
 */
std::ostream &operator << (std::ostream &os, const Vector3D &vector);
/**
 * Output streamer.
 *
 * Vectors are written as "x:y".
 *
 * \param [in,out] os The stream.
 * \param [in] vector The vector to stream
 * \return The stream.
 */
std::ostream &operator << (std::ostream &os, const Vector2D &vector);

/**
 * Input streamer.
 *
 * Vectors are expected to be in the form "x:y:z".
 *
 * \param [in,out] is The stream.
 * \param [in] vector The vector.
 * \returns The stream.
 */
std::istream &operator >> (std::istream &is, Vector3D &vector);
/**
 * Input streamer.
 *
 * Vectors are expected to be in the form "x:y".
 *
 * \param [in,out] is The stream.
 * \param [in] vector The vector.
 * \returns The stream.
 */
std::istream &operator >> (std::istream &is, Vector2D &vector);


/**
 * \relates Vector3D
 * Vector alias typedef for compatibility with mobility models
 */
typedef Vector3D Vector;



class SumoTclParser : public SumoParser{

public:
	/**
	 * Type to maintain line parsed and its values
	 */
	struct ParseResult
	{
	  std::vector<std::string> tokens; //!< tokens from a line
	  std::vector<int> ivals;     //!< int values for each tokens
	  std::vector<bool> has_ival; //!< points if a tokens has an int value
	  std::vector<double> dvals;  //!< double values for each tokens
	  std::vector<bool> has_dval; //!< points if a tokens has a double value
	  std::vector<std::string> svals;  //!< string value for each token
	};
	/**
	 * Keeps last movement schedule. If new movement occurs during
	 * a current one, node stopping must be cancels (stored in a proper
	 * event ID), actually reached point must be calculated and new
	 * velocity must be calculated in accordance with actually reached
	 * destination.
	 */
	struct DestinationPoint
	{
	  Vector m_startPosition;     //!< Start position of last movement
	  Vector m_speed;             //!< Speed of the last movement (needed to derive reached destination at next schedule = start + velocity * actuallyTravelled)
	  Vector m_finalPosition;     //!< Final destination to be reached before next schedule. Replaced with actually reached if needed.
	  //EventId m_stopEvent;        //!< Event scheduling node's stop. May be canceled if needed.
	  double m_travelStartTime;   //!< Travel start time is needed to calculate actually traveled time
	  double m_targetArrivalTime; //!< When a station arrives to a destination
	  DestinationPoint () :
	    m_startPosition (Vector (0,0,0)),
	    m_speed (Vector (0,0,0)),
	    m_finalPosition (Vector (0,0,0)),
	    m_travelStartTime (0),
	    m_targetArrivalTime (0)
	  {};
	};

	/**
	 * Parses a line of ns2 mobility
	 */
	static ParseResult ParseNs2Line (const std::string& str);

	/** 
	 * Put out blank spaces at the start and end of a line
	 */
	static std::string TrimNs2Line (const std::string& str);

	/**
	 * Checks if a string represents a number or it has others characters than digits an point.
	 */
	static bool IsNumber (const std::string& s);

	/**
	 * Check if s string represents a numeric value
	 * \param str string to check
	 * \param ret numeric value to return
	 * \return true if string represents a numeric value
	 */
	template<class T>
	static bool IsVal (const std::string& str, T& ret);

	/**
	 * Checks if the value between brackets is a correct nodeId number
	 */ 
	static bool HasNodeIdNumber (std::string str);

	/** 
	 * Gets nodeId number in string format from the string like $node_(4)
	 */
	static std::string GetNodeIdFromToken (std::string str);

	/** 
	 * Get node id number in int format
	 */
	static int GetNodeIdInt (ParseResult pr);

	/**  
	 * Get node id number in string format
	 */
	static std::string GetNodeIdString (ParseResult pr);

	/**
	 * Add one coord to a vector position
	 */
	static Vector SetOneInitialCoord (Vector actPos, std::string& coord, double value);

	/** 
	 * Check if this corresponds to a line like this: $node_(0) set X_ 123
	 */
	static bool IsSetInitialPos (ParseResult pr);

	/** 
	 * Check if this corresponds to a line like this: $ns_ at 1 "$node_(0) setdest 2 3 4"
	 */
	static bool IsSchedSetPos (ParseResult pr);

	/**
	 * Check if this corresponds to a line like this: $ns_ at 1 "$node_(0) set X_ 2"
	 */
	static bool IsSchedMobilityPos (ParseResult pr);
	void Initial(std::string filename);
	void ConfigNodesMovement(std::string filename);
	SUMOTraceMap parse(std::string filename);
	static void Init();
	static void DeleteInstance();
	static SumoTclParser* Instance() { return inst_; }
private:
    static SumoTclParser* inst_;
    SumoTclParser();
    SumoTclParser( const SumoTclParser& );             // disabled
    SumoTclParser& operator=( const SumoTclParser& );  // disabled
};

inline SumoTclParser& SumoTclPsr()
{
  return *SumoTclParser::Instance();
}

#endif