#pragma once
/**
 * \file autons.hpp
 * Declarations for the example autonomous routines in autons.cpp.
 * (In your project this lives in include/, next to main.h.)
 *
 * Add a declaration here whenever you write a new routine in autons.cpp,
 * then add it to the selector list in main.cpp — that's the whole loop.
 */

void simpleAuton();   ///< EZ-style: tile-based relative motions
void pointAuton();    ///< LemLib-style: absolute field coordinates
void chainedAuton();  ///< motion chaining: flow through waypoints at speed
void asyncAuton();    ///< do things WHILE driving (waitUntil / cancel)
void pathAuton();     ///< pure pursuit along a Bezier path
void metricAuton();   ///< same API, metric literals
