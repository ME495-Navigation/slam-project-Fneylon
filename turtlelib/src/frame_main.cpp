#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

using namespace std;
int main()
{
    // Initalize the svg class
    turtlelib::BuildSVG svg;

    // std::istream ss1;
    // std::istream ss2;
    // Initalize the transforms to compute
    turtlelib::Transform2D Twa; // Frame a in the world frame

    turtlelib::Transform2D Tab;
    turtlelib::Transform2D Tbc;

    turtlelib::Transform2D Tba;
    turtlelib::Transform2D Tcb;

    turtlelib::Transform2D Tac;
    turtlelib::Transform2D Tca;

    cout<< "Enter Transform T_{a,b}\n";
    cin >> Tab;

    cout << "Enter Transform T_{b,c}\n";
    cin >> Tbc;

    //Compute the Frame Transformations:
    Tba = Tab.inv();
    Tcb = Tbc.inv();

    Tac = Tab * Tbc;
    Tca = Tcb * Tba;

    // Print out the Frames: 
    cout << "T_{a,b}: " << Tab << "\n";
    cout << "T_{b,a}: " << Tba << "\n";
    cout << "T_{b,c}: " << Tbc << "\n";
    cout << "T_{c,b}: " << Tcb << "\n";
    cout << "T_{a,c}: " << Tac << "\n";
    cout << "T_{c,a}: " << Tca << "\n";

    // Draw the Frames: 
    svg.Draw(Twa, "{a}");
    svg.Draw(Tab, "T_{a,b}");
    svg.Draw(Tba, "T_{b,a}");
    svg.Draw(Tbc, "T_{b,c}");
    svg.Draw(Tcb, "T_{c,b}");
    svg.Draw(Tac, "T_{a,c}");
    svg.Draw(Tca, "T_{c,a}");

    // Export the SVG
    

    // Prompt the user to enter a point pa in frame a
    turtlelib::Point2D Pa;
    cout<<"Enter a point in frame a:\n";
    cin>>Pa;

    // Draw the point in frame a
    svg.Draw(Pa, "purple");

    // Draw the point in frame b
    turtlelib::Point2D Pb;
    Pb = Tba(Pa);
    svg.Draw(Pb, "brown");

    // Draw the point in frame c
    turtlelib::Point2D Pc;
    Pc = Tca(Pa);
    svg.Draw(Pc, "orange");

    // Export the SVG
    // svg.ExportSVG("frames.svg");

    cout << Pa << "\n";
    cout << Pb << "\n";
    cout << Pc << "\n";

    // Prompt the user to enter a vector vb in frame b
    turtlelib::Point2D P0;
    P0.x = 0.0;
    P0.y = 0.0;

    turtlelib::Vector2D Vb;
    turtlelib::Vector2D Vb_hat;
    cout << "Enter a vector in frame b:\n";
    cin >> Vb;

    // Normalize the vector
    Vb_hat.x = Vb.x/sqrt(pow(Vb.x,2) + pow(Vb.y,2));
    Vb_hat.y = Vb.y/sqrt(pow(Vb.x,2) + pow(Vb.y,2));
    cout << "Vb_hat: " << Vb_hat << "\n";

    // Draw the vector in frame b
    turtlelib::Point2D Vb_hat_point;
    Vb_hat_point.x = Vb_hat.x;
    Vb_hat_point.y = Vb_hat.y;
    svg.Draw(Vb_hat_point, P0, "brown");

    // Output vb in frame a
    turtlelib::Vector2D Va;
    turtlelib::Point2D Va_point;
    Va = Tab(Vb);
    Va_point.x = Va.x;
    Va_point.y = Va.y;
    svg.Draw(Va_point, P0, "purple");
    cout << "Va: " << Va << "\n";


    // Draw the vector in frame c
    turtlelib::Vector2D Vc;
    turtlelib::Point2D Vc_point;
    Vc = Tca(Va);
    Vc_point.x = Vc.x;
    Vc_point.y = Vc.y;
    svg.Draw(Vc_point, P0, "orange");
    cout << "Vc: " << Vc << "\n";

    // Enter the Twist in frame b
    turtlelib::Twist2D Vb_twist;
    cout << "Enter a twist in frame b:\n";
    cin >> Vb_twist;

    //Draw the twist in frame a
    turtlelib::Twist2D Va_twist;
    Va_twist = Tab(Vb_twist);
    cout << "Va_twist: " << Va_twist << "\n";

    cout << "Vb_twist: " << Vb_twist << "\n";

    // Draw the twist in frame c
    turtlelib::Twist2D Vc_twist;
    Vc_twist = Tca(Va_twist);
    cout << "Vc_twist: " << Vc_twist << "\n";

    svg.ExportSVG("/tmp/frames.svg");

}