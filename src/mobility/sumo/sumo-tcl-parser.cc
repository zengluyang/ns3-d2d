#include "basic-models/mobility/sumo-tcl-parser.h"
#include "simcore/dassert.h"
using namespace std;





Vector3D::Vector3D (double _x, double _y, double _z)
  : x (_x),
    y (_y),
    z (_z)
{
}

Vector3D::Vector3D ()
  : x (0.0),
    y (0.0),
    z (0.0)
{
}

Vector2D::Vector2D (double _x, double _y)
  : x (_x),
    y (_y)
{
}


Vector2D::Vector2D ()
  : x (0.0),
    y (0.0)
{
}

double
CalculateDistance (const Vector3D &a, const Vector3D &b)
{
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dz = b.z - a.z;
  double distance = std::sqrt (dx * dx + dy * dy + dz * dz);
  return distance;
}
double 
CalculateDistance (const Vector2D &a, const Vector2D &b)
{
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double distance = std::sqrt (dx * dx + dy * dy);
  return distance;
}

std::ostream &operator << (std::ostream &os, const Vector3D &vector)
{
  os << vector.x << ":" << vector.y << ":" << vector.z;
  return os;
}
std::istream &operator >> (std::istream &is, Vector3D &vector)
{
  char c1, c2;
  is >> vector.x >> c1 >> vector.y >> c2 >> vector.z;
  if (c1 != ':' ||
      c2 != ':')
    {
      is.setstate (std::ios_base::failbit);
    }
  return is;
}
std::ostream &operator << (std::ostream &os, const Vector2D &vector)
{
  os << vector.x << ":" << vector.y;
  return os;
}
std::istream &operator >> (std::istream &is, Vector2D &vector)
{
  char c1;
  is >> vector.x >> c1 >> vector.y;
  if (c1 != ':')
    {
      is.setstate (std::ios_base::failbit);
    }
  return is;
}

SumoTclParser* SumoTclParser::inst_ = 0;


SumoTclParser::SumoTclParser() 
{

}

void SumoTclParser::Init() {
  SumoTclParser::inst_ = new SumoTclParser();
}

void SumoTclParser::DeleteInstance()
{
    delete inst_;
    inst_ = 0;
}

SumoTclParser::ParseResult
SumoTclParser::ParseNs2Line (const std::string& str)
{
  ParseResult ret;
  std::istringstream s;
  std::string line;

  // ignore comments (#)
  size_t pos_sharp = str.find_first_of ('#');
  if (pos_sharp != std::string::npos)
    {
      line = str.substr (0, pos_sharp);
    }
  else
    {
      line = str;
    }

  line = TrimNs2Line (line);

  // If line hasn't a correct node Id
  if (!HasNodeIdNumber (line))
    {
      return ret;
    }

  s.str (line);

  while (!s.eof ())
    {
      std::string x;
      s >> x;
      if (x.length () == 0)
        {
          continue;
        }
      ret.tokens.push_back (x);
      int ii (0);
      double d (0);
      if (HasNodeIdNumber (x))
        {
          x = GetNodeIdFromToken (x);
        }
      ret.has_ival.push_back (IsVal<int> (x, ii));
      ret.ivals.push_back (ii);
      ret.has_dval.push_back (IsVal<double> (x, d));
      ret.dvals.push_back (d);
      ret.svals.push_back (x);
    }

  size_t tokensLength   = ret.tokens.size ();                 // number of tokens in line
  size_t lasTokenLength = ret.tokens[tokensLength - 1].size (); // length of the last token

  // if it is a scheduled set _[XYZ] or a setdest I need to remove the last "
  // and re-calculate values
  if ( (tokensLength == 7 || tokensLength == 8)
       && (ret.tokens[tokensLength - 1][lasTokenLength - 1] == '"') )
    {

      // removes " from the last position
      ret.tokens[tokensLength - 1] = ret.tokens[tokensLength - 1].substr (0,lasTokenLength - 1);

      std::string x;
      x = ret.tokens[tokensLength - 1];

      if (HasNodeIdNumber (x))
        {
          x = GetNodeIdFromToken (x);
        }

      // Re calculate values
      int ii (0);
      double d (0);
      ret.has_ival[tokensLength - 1] = IsVal<int> (x, ii);
      ret.ivals[tokensLength - 1] = ii;
      ret.has_dval[tokensLength - 1] = IsVal<double> (x, d);
      ret.dvals[tokensLength - 1] = d;
      ret.svals[tokensLength - 1] = x;

    }
  else if ( (tokensLength == 9 && ret.tokens[tokensLength - 1] == "\"")
            || (tokensLength == 8 && ret.tokens[tokensLength - 1] == "\""))
    {
      // if the line has the " character in this way: $ns_ at 1 "$node_(0) setdest 2 2 1  "
      // or in this: $ns_ at 4 "$node_(0) set X_ 2  " we need to ignore this last token

      ret.tokens.erase (ret.tokens.begin () + tokensLength - 1);
      ret.has_ival.erase (ret.has_ival.begin () + tokensLength - 1);
      ret.ivals.erase (ret.ivals.begin () + tokensLength - 1);
      ret.has_dval.erase (ret.has_dval.begin () + tokensLength - 1);
      ret.dvals.erase (ret.dvals.begin () + tokensLength - 1);
      ret.svals.erase (ret.svals.begin () + tokensLength - 1);

    }



  return ret;
}


std::string
SumoTclParser::TrimNs2Line (const std::string& s)
{
  std::string ret = s;

  while (ret.size () > 0 && isblank (ret[0]))
    {
      ret.erase (0, 1);    // Removes blank spaces at the begining of the line
    }

  while (ret.size () > 0 && (isblank (ret[ret.size () - 1]) || (ret[ret.size () - 1] == ';')))
    {
      ret.erase (ret.size () - 1, 1); // Removes blank spaces from at end of line
    }

  return ret;
}


bool
SumoTclParser::IsNumber (const std::string& s)
{
  char *endp;
  double v = strtod (s.c_str (), &endp); // declared with warn_unused_result
  return endp == s.c_str () + s.size ();
}


template<class T>
bool
SumoTclParser::IsVal (const std::string& str, T& ret)
{
  if (str.size () == 0)
    {
      return false;
    }
  else if (IsNumber (str))
    {
      std::string s2 = str;
      std::istringstream s (s2);
      s >> ret;
      return true;
    }
  else
    {
      return false;
    }
}


bool
SumoTclParser::HasNodeIdNumber (std::string str)
{

  // find brackets
  std::string::size_type startNodeId = str.find_first_of ("("); // index of left bracket
  std::string::size_type endNodeId   = str.find_first_of (")"); // index of right bracket

  // Get de nodeId in a string and in a int value
  std::string nodeId;     // node id

  // if no brackets, continue!
  if (startNodeId == std::string::npos || endNodeId == std::string::npos)
    {
      return false;
    }

  nodeId = str.substr (startNodeId + 1, endNodeId - (startNodeId + 1)); // set node id

  //   is number              is integer                                       is not negative
  if (IsNumber (nodeId) && (nodeId.find_first_of (".") == std::string::npos) && (nodeId[0] != '-'))
    {
      return true;
    }
  else
    {
      return false;
    }
}


std::string
SumoTclParser::GetNodeIdFromToken (std::string str)
{
  if (HasNodeIdNumber (str))
    {
      // find brackets
      std::string::size_type startNodeId = str.find_first_of ("(");     // index of left bracket
      std::string::size_type endNodeId   = str.find_first_of (")");     // index of right bracket

      return str.substr (startNodeId + 1, endNodeId - (startNodeId + 1)); // set node id
    }
  else
    {
      return "";
    }
}


int
SumoTclParser::GetNodeIdInt (ParseResult pr)
{
  int result = -1;
  switch (pr.tokens.size ())
    {
    case 4:   // line like $node_(0) set X_ 11
      result = pr.ivals[0];
      break;
    case 7:   // line like $ns_ at 4 "$node_(0) set X_ 28"
      result = pr.ivals[3];
      break;
    case 8:   // line like $ns_ at 1 "$node_(0) setdest 2 3 4"
      result = pr.ivals[3];
      break;
    default:
      result = -1;
    }
  return result;
}

// Get node id number in string format
std::string
SumoTclParser::GetNodeIdString (ParseResult pr)
{
  switch (pr.tokens.size ())
    {
    case 4:   // line like $node_(0) set X_ 11
      return pr.svals[0];
      break;
    case 7:   // line like $ns_ at 4 "$node_(0) set X_ 28"
      return pr.svals[3];
      break;
    case 8:   // line like $ns_ at 1 "$node_(0) setdest 2 3 4"
      return pr.svals[3];
      break;
    default:
      return "";
    }
}


Vector
SumoTclParser::SetOneInitialCoord (Vector position, std::string& coord, double value)
{

  // set the position for the coord.
  if (coord == NS2_X_COORD)
    {
      position.x = value;
    }
  else if (coord == NS2_Y_COORD)
    {
      position.y = value;
    }
  else if (coord == NS2_Z_COORD)
    {
      position.z = value;
    }
  return position;
}


bool
SumoTclParser::IsSetInitialPos (ParseResult pr)
{
  //        number of tokens         has $node_( ?                        has "set"           has doble for position?
  return pr.tokens.size () == 4 && HasNodeIdNumber (pr.tokens[0]) && pr.tokens[1] == NS2_SET && pr.has_dval[3]
         // coord name is X_, Y_ or Z_ ?
         && (pr.tokens[2] == NS2_X_COORD || pr.tokens[2] == NS2_Y_COORD || pr.tokens[2] == NS2_Z_COORD);

}


bool
SumoTclParser::IsSchedSetPos (ParseResult pr)
{
  //      correct number of tokens,    has $ns_                   and at
  return pr.tokens.size () == 7 && pr.tokens[0] == NS2_NS_SCH && pr.tokens[1] == NS2_AT
         && pr.tokens[4] == NS2_SET && pr.has_dval[2] && pr.has_dval[3]   // has set and double value for time and nodeid
         && ( pr.tokens[5] == NS2_X_COORD || pr.tokens[5] == NS2_Y_COORD || pr.tokens[5] == NS2_Z_COORD) // has X_, Y_ or Z_?
         && pr.has_dval[2]; // time is a number
}

bool
SumoTclParser::IsSchedMobilityPos (ParseResult pr)
{
  //     number of tokens      and    has $ns_                and    has at
  return pr.tokens.size () == 8 && pr.tokens[0] == NS2_NS_SCH && pr.tokens[1] == NS2_AT
         //    time             x coord          y coord          velocity are numbers?
         && pr.has_dval[2] && pr.has_dval[5] && pr.has_dval[6] && pr.has_dval[7]
         && pr.tokens[4] == NS2_SETDEST; // and has setdest

}

void 
SumoTclParser::Initial(std::string filename)
{
    std::ifstream file (filename.c_str (), std::ios::in);
    double time_= 0.0;
    if (file.is_open ())
      {
        vector<Vechile> cars;
        while (!file.eof () )
          {
            int iNodeId = 0;
            std::string nodeId;
            std::string line;
            Vechile m_vechile;
            getline (file, line);

            // ignore empty lines
            if (line.empty ())
              {
                continue;
              }

            SumoTclParser::ParseResult pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens

            // Check if the line corresponds with setting the initial
            // node positions
            if (pr.tokens.size () != 4)
              {
                continue;
              }

            // Get the node Id
            nodeId  = SumoTclParser::GetNodeIdString (pr);
            iNodeId = SumoTclParser::GetNodeIdInt (pr);
            if (iNodeId == -1)
              {
                continue;
              }
            m_vechile.id = iNodeId;
            m_vechile.time = SimTime(0.0);
            m_vechile.velocity = 0.0;
            if(pr.tokens[2]=="X_")
            {
              m_vechile.x = pr.dvals[3];
              getline (file, line);
              SumoTclParser::ParseResult pr1 = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
              if(pr1.tokens[2]=="Y_")
              {
                m_vechile.y = pr1.dvals[3];
                getline (file, line);
                SumoTclParser::ParseResult pr2 = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
                if(pr2.tokens[2]=="Z_")
                {
                  m_vechile.z = pr2.dvals[3];
                  cars.push_back(m_vechile);
                }
                else
                {
                  std::cout<<"Wrong position Label!"<<std::endl;
                }
              }
            }

            /*
             * In this case a initial position is being seted
             * line like $node_(0) set X_ 151.05190721688197
             */
             // std::cout<<"INIT "<<iNodeId<<" "<<pr.tokens[2]<<" "<<pr.dvals[3]<<std::endl;

          }
        file.close ();
        trace.insert(make_pair(SimTime(time_),cars));
        std::cout<<"INIT "<<time_<<" "<<cars.size()<<std::endl;
    }  
}

void 
SumoTclParser::ConfigNodesMovement(std::string filename)
{
  std::ifstream file (filename.c_str (), std::ios::in);
  if (file.is_open ())
  {
    double m_time = 0.0;
    bool m_break = true;
    SumoTclParser::ParseResult pr;
    while (!file.eof () )
      {
      bool m_first = true;
      vector<Vechile> cars;
      if(m_break == false)
      {
                Vechile m_vechile;
                int iNodeId = 0;
                std::string line;
                /*
                 * In this case a new waypoint is added
                 * line like $ns_ at 1 "$node_(0) setdest 2 3 4"
                 */
                if (SumoTclParser::IsSchedMobilityPos (pr))
                  {
                    m_vechile.id = iNodeId;
                    m_vechile.time = SimTime(pr.dvals[2]);
                    m_vechile.velocity = pr.dvals[7];
                    m_vechile.x = pr.dvals[5];
                    m_vechile.y = pr.dvals[6];
                    m_vechile.z = 0.0;
                    cars.push_back(m_vechile);
                  }


                /*
                 * Scheduled set position
                 * line like $ns_ at 4.634906291962 "$node_(0) set X_ 28.675920486450"
                 */
                else if (SumoTclParser::IsSchedSetPos (pr))                 //假设所有的初始化，都是xyz挨着的
                  {
                    if(pr.tokens[5]=="X_")
                    {
                      m_vechile.x = pr.dvals[6];
                      getline (file, line);
                      SumoTclParser::ParseResult pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
                      if(SumoTclParser::IsSchedSetPos (pr) && pr.tokens[5]=="Y_")
                      {
                        m_vechile.y = pr.dvals[6];
                        getline (file, line);
                        SumoTclParser::ParseResult pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
                        if(SumoTclParser::IsSchedSetPos (pr) && pr.tokens[5]=="Z_")
                        {
                          m_vechile.z = pr.dvals[6];
                          cars.push_back(m_vechile);
                        }
                        else
                        {
                          std::cout<<"Wrong position Label!"<<std::endl;
                        }
                      }
                    }
                  }
                else
                  {
                    std::cout<<"No this condition!"<<std::endl;
                  }
        m_break = true;
      }
      while(true&&(!file.eof ()) ){
          int iNodeId = 0;
          double time_ = 0.0;
          std::string nodeId;
          std::string line;
          Vechile m_vechile;
          getline (file, line);

          // ignore empty lines
          if (line.empty ())
            {
              continue;
            }

          pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens

          // Check if the line corresponds with one of the three types of line
          if (pr.tokens.size () != 4 && pr.tokens.size () != 7 && pr.tokens.size () != 8)
            {
              continue;
            }

          // Get the node Id
          nodeId  = SumoTclParser::GetNodeIdString (pr);
          iNodeId = SumoTclParser::GetNodeIdInt (pr);
          if (iNodeId == -1)
            {
              continue;
            }

          // This is a scheduled event, so time at should be present
          double at;

          if (!SumoTclParser::IsNumber (pr.tokens[2]))
            {
              continue;
            }

          at = pr.dvals[2]; // set time at

          if ( at < 0 )
            {
              continue;
            }
          time_ = at;
          if(m_first == true)
          {
            m_time = time_;
            m_first = false;
          }
          if(m_time == time_)
          {
                /*
                 * In this case a new waypoint is added
                 * line like $ns_ at 1 "$node_(0) setdest 2 3 4"
                 */
                if (SumoTclParser::IsSchedMobilityPos (pr))
                  {
                    m_vechile.id = iNodeId;
                    m_vechile.time = SimTime(pr.dvals[2]);
                    m_vechile.velocity = pr.dvals[7];
                    m_vechile.x = pr.dvals[5];
                    m_vechile.y = pr.dvals[6];
                    m_vechile.z = 0.0;
                    cars.push_back(m_vechile);
                    // std::cout<<"NEWWAY "<<iNodeId<<" "<<at<<" "<<pr.dvals[5]<<" "<<pr.dvals[6]<<" "<<pr.dvals[7]<<std::endl;
                  }


                /*
                 * Scheduled set position
                 * line like $ns_ at 4.634906291962 "$node_(0) set X_ 28.675920486450"
                 */
                else if (SumoTclParser::IsSchedSetPos (pr))                 //假设所有的初始化，都是xyz挨着的
                  {
                    if(pr.tokens[5]=="X_")
                    {
                      m_vechile.x = pr.dvals[6];
                      getline (file, line);
                      SumoTclParser::ParseResult pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
                      if(SumoTclParser::IsSchedSetPos (pr) && pr.tokens[5]=="Y_")
                      {
                        m_vechile.y = pr.dvals[6];
                        getline (file, line);
                        SumoTclParser::ParseResult pr = SumoTclParser::ParseNs2Line (line); // Parse line and obtain tokens
                        if(SumoTclParser::IsSchedSetPos (pr) && pr.tokens[5]=="Z_")
                        {
                          m_vechile.z = pr.dvals[6];
                          cars.push_back(m_vechile);
                        }
                        else
                        {
                          std::cout<<"Wrong position Label!"<<std::endl;
                        }
                      }
                    }
                    // std::cout<<"SETPOS "<<iNodeId<<" "<<at<<" "<<pr.tokens[5]<<" "<<pr.dvals[6]<<std::endl;
                  }
                else
                  {
                    std::cout<<"No this condition!"<<std::endl;
                  }
          }
         else
          {
            trace.insert(make_pair(SimTime(m_time),cars));
            std::cout<<"SETPOS "<<time_<<" "<<cars.size()<<std::endl;
            m_time = time_;
            m_break = false;
            break;
          }

        }
        if(file.eof ())
        {
          trace.insert(make_pair(SimTime(m_time),cars));
          //std::cout<<"1000a"<<std::endl;
        }
      }
    file.close ();
  }
}


SUMOTraceMap SumoTclParser::parse(std::string filename){
  std::cout<<filename<<std::endl;
  Initial(filename);
  ConfigNodesMovement(filename);
  findMinMax();
  return trace;
}