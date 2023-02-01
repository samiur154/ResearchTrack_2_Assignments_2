/**
 * \file position_service.cpp
 * \brief This files creates a service server for definig the robot's pose
 * \author SAMIUR RAHMAN
 * \version 1.0
 * \date 29/07/2022
 * 
 * \details
 * 
 * Services : <BR>
 * 		/position_server
 * 
 *  Description :
 *  
 *  This node promotes a   position  service. When the service is used,
     a request providing the x and y positio
 *  
 *  is used to generate a random position between x or y min and x or y
 *  max.
* 
 */
 
#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
 * \brief Generate a bounded random number
 *
 * \param M (double): Lower bound of the generated random number.
 * \param N (double): Upper bound of the generated random number.
 *
 * \return randMToN (double):
 *   random number, between M and N.
 * 
 *  Description :
 *  
 *  This function genearte a random number between M and N 
*/

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/** 
 * bool myrandom (res, req)
 * 
 * \brief it is a Function where get the position request and call the function which generate the random position
 * 
 * \param req: the range of minimum and maximum values that the random position must have in the request made from another node
 * 
 * \param res: the reply of the server with the x, y and theta coordinates of the target
 * 
 * \return true: it notifies that it has generated the random position
 * 
 * description:
 *     obtains the interval's parameters from the request, then calls randMToN to obtain number in that interval.
 * 
 */

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


/** 
 * int main(int argc, char **argv)
 * 
 * \brief The main function of the node 
 * 
 * \param argc: the number of arguent passed as parameters
 * 
 * \param argv: the vector of string containing each argument
 * 
 * \return 0 when the program has finished
 * 
 * description:
 *      Main function that initialize the server
 * 
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
