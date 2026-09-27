/////////////////////////////////////////////////////////////////////////////
// Name:        netlist.cpp
// Purpose:     SPICE netlist parsing & processing
// Author:      Francesco Montorsi
// Created:     30/05/2010
// Copyright:   (c) 2010 Francesco Montorsi
// Licence:     GPL licence
/////////////////////////////////////////////////////////////////////////////

// ============================================================================
// declarations
// ============================================================================

// ----------------------------------------------------------------------------
// headers
// ----------------------------------------------------------------------------
 
#include <wx/wx.h>
#include <wx/wfstream.h>
#include <wx/sstream.h>
#include <wx/tokenzr.h>
#include <wx/filename.h>

#include <string.h>
#include <stdio.h>

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <map>
#include <queue>

#include <boost/graph/kamada_kawai_spring_layout.hpp>
#include <boost/graph/circle_layout.hpp>

#include "netlist.h"
#include "devices.h"

namespace
{
struct svWireTerminal
{
    wxPoint node;
    wxPoint escape;
    wxPoint direction;
    bool externalPin;
};

struct svGridPointLess
{
    bool operator()(const wxPoint& a, const wxPoint& b) const
    {
        if (a.x != b.x)
            return a.x < b.x;
        return a.y < b.y;
    }
};

struct svGridEdge
{
    wxPoint a;
    wxPoint b;
};

struct svGridEdgeLess
{
    bool operator()(const svGridEdge& lhs, const svGridEdge& rhs) const
    {
        svGridPointLess less;
        if (less(lhs.a, rhs.a))
            return true;
        if (less(rhs.a, lhs.a))
            return false;
        return less(lhs.b, rhs.b);
    }
};

typedef std::set<svGridEdge, svGridEdgeLess> svGridEdgeSet;
typedef std::set<wxPoint, svGridPointLess> svGridPointSet;

struct svRoutingObstacle
{
    wxRect bounds;
    const svBaseDevice* device;
};

typedef std::vector<svRoutingObstacle> svRoutingObstacleArray;

static void drawGridLine(wxGraphicsContext* gc, const wxPoint& from,
                         const wxPoint& to, unsigned int gridSize)
{
    if (from == to)
        return;

    drawLine(gc, from * gridSize, to * gridSize);
}

static bool isPowerNodeName(const svNode& node)
{
    std::string upper(node);
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char ch) { return (char)std::toupper(ch); });

    return upper == "VCC" || upper == "VDD" || upper == "VSS" ||
           upper == "VEE" || upper == "VBAT" || upper == "VREF" ||
           upper == "POWER" || upper == "+V" || upper == "-V" ||
           upper.find("VCC") == 0 || upper.find("VDD") == 0;
}

static bool isUsefulSignalLabel(const svNode& node)
{
    std::string upper(node);
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char ch) { return (char)std::toupper(ch); });

    return upper == "IN" || upper == "INPUT" || upper == "VIN" ||
           upper == "OUT" || upper == "OUTPUT" || upper == "VOUT" ||
           upper.find("VIN") == 0 || upper.find("VOUT") == 0;
}

static bool isInputLikeNode(const svNode& node)
{
    std::string upper(node);
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char ch) { return (char)std::toupper(ch); });
    return upper == "IN" || upper == "INPUT" || upper == "VIN" ||
           upper.find("VIN") == 0 || upper.find("INPUT") == 0;
}

static bool isOutputLikeNode(const svNode& node)
{
    std::string upper(node);
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char ch) { return (char)std::toupper(ch); });
    return upper == "OUT" || upper == "OUTPUT" || upper == "VOUT" ||
           upper.find("VOUT") == 0 || upper.find("OUTPUT") == 0;
}

static wxPoint getTerminalEscape(const svBaseDevice* dev, unsigned int nodeIdx)
{
    return dev->getGridPosition() +
           dev->getRelativeGridNodePosition(nodeIdx) +
           dev->getRelativeGridNodeDirection(nodeIdx);
}

static void alignTerminalEscape(svBaseDevice* dev, unsigned int nodeIdx,
                                const wxPoint& desiredEscape)
{
    dev->setGridPosition(
        desiredEscape -
        dev->getRelativeGridNodePosition(nodeIdx) -
        dev->getRelativeGridNodeDirection(nodeIdx));
}

static int orderlyBranchLaneOffset(unsigned int branch)
{
    // One grid is not enough once the rendered symbol, its horizontal
    // annotation and any ground/power decoration are taken into account.
    // Two grids is the nominal lane pitch; the collision test below can skip
    // additional lanes when a wider label or neighboring branch needs it.
    return (int)branch * 2;
}

struct svPlacementRect
{
    int left;
    int top;
    int right;
    int bottom;
};

static void includePlacementPoint(svPlacementRect* rect, const wxPoint& pt)
{
    rect->left = std::min(rect->left, pt.x);
    rect->right = std::max(rect->right, pt.x);
    rect->top = std::min(rect->top, pt.y);
    rect->bottom = std::max(rect->bottom, pt.y);
}

static svPlacementRect getPlacementRect(const svBaseDevice* dev)
{
    const wxPoint pos = dev->getGridPosition();
    svPlacementRect rect;
    rect.left = pos.x + dev->getLeftmostGridNodePosition();
    rect.right = pos.x + dev->getRightmostGridNodePosition();
    rect.top = pos.y + dev->getTopmostGridNodePosition();
    rect.bottom = pos.y + dev->getBottommostGridNodePosition();

    for (size_t nodeIdx=0; nodeIdx<dev->getNodesCount(); nodeIdx++)
    {
        const wxPoint node =
            pos + dev->getRelativeGridNodePosition((unsigned int)nodeIdx);
        includePlacementPoint(&rect, node);

        const wxPoint direction =
            dev->getRelativeGridNodeDirection((unsigned int)nodeIdx);
        if (dev->getNode((unsigned int)nodeIdx) == svGroundNode)
        {
            // Ground is drawn after placement: one complete grid stub followed
            // by the symbol itself. Reserving the ground origin is enough at
            // grid resolution; the art itself is less than one grid tall/wide.
            includePlacementPoint(&rect, node + direction);
        }
        else if (isPowerNodeName(dev->getNode((unsigned int)nodeIdx)))
        {
            // Supply terminals receive an outward global-rail marker and text.
            includePlacementPoint(&rect, node + direction);
        }
    }

    if (dev->getSPICEid() == 0)
    {
        // External pins report a zero-sized logical box even though the pin
        // circle/stem is visible. Reserve one grid around the anchor. Pin text
        // is kept clear by placing pins outside the core, below.
        rect.left -= 1;
        rect.right += 1;
        rect.top -= 1;
        rect.bottom += 1;
    }

    return rect;
}

static svPlacementRect getBodyPlacementRect(const svBaseDevice* dev)
{
    const wxPoint pos = dev->getGridPosition();
    svPlacementRect rect;
    rect.left = pos.x + dev->getLeftmostGridNodePosition();
    rect.right = pos.x + dev->getRightmostGridNodePosition();
    rect.top = pos.y + dev->getTopmostGridNodePosition();
    rect.bottom = pos.y + dev->getBottommostGridNodePosition();
    return rect;
}

static bool gridSegmentIntersectsPlacementRect(
    const wxPoint& from,
    const wxPoint& to,
    const svPlacementRect& rect)
{
    if (from.x == to.x)
    {
        return from.x >= rect.left && from.x <= rect.right &&
               std::max(from.y, to.y) >= rect.top &&
               std::min(from.y, to.y) <= rect.bottom;
    }
    if (from.y == to.y)
    {
        return from.y >= rect.top && from.y <= rect.bottom &&
               std::max(from.x, to.x) >= rect.left &&
               std::min(from.x, to.x) <= rect.right;
    }
    return false;
}

static bool terminalLeadHitsDeviceBody(const svBaseDevice* from,
                                       const svBaseDevice* body)
{
    const svPlacementRect bodyRect = getBodyPlacementRect(body);
    for (size_t nodeIdx=0; nodeIdx<from->getNodesCount(); nodeIdx++)
    {
        const wxPoint node =
            from->getGridPosition() +
            from->getRelativeGridNodePosition((unsigned int)nodeIdx);
        const wxPoint escape =
            node + from->getRelativeGridNodeDirection((unsigned int)nodeIdx);
        if (gridSegmentIntersectsPlacementRect(node, escape, bodyRect))
            return true;
    }
    return false;
}

static bool gridSegmentContainsGridPoint(const wxPoint& from,
                                         const wxPoint& to,
                                         const wxPoint& point)
{
    if (from.x == to.x)
    {
        return point.x == from.x &&
               point.y >= std::min(from.y, to.y) &&
               point.y <= std::max(from.y, to.y);
    }
    if (from.y == to.y)
    {
        return point.y == from.y &&
               point.x >= std::min(from.x, to.x) &&
               point.x <= std::max(from.x, to.x);
    }
    return false;
}

static bool terminalLeadHitsGroundOrigin(const svBaseDevice* from,
                                         const svBaseDevice* groundOwner)
{
    std::vector<wxPoint> groundOrigins;
    for (size_t groundIdx=0; groundIdx<groundOwner->getNodesCount(); groundIdx++)
    {
        if (groundOwner->getNode((unsigned int)groundIdx) != svGroundNode)
            continue;

        const wxPoint node =
            groundOwner->getGridPosition() +
            groundOwner->getRelativeGridNodePosition((unsigned int)groundIdx);
        groundOrigins.push_back(
            node + groundOwner->getRelativeGridNodeDirection(
                (unsigned int)groundIdx));
    }

    if (groundOrigins.empty())
        return false;

    for (size_t nodeIdx=0; nodeIdx<from->getNodesCount(); nodeIdx++)
    {
        const wxPoint node =
            from->getGridPosition() +
            from->getRelativeGridNodePosition((unsigned int)nodeIdx);
        const wxPoint escape =
            node + from->getRelativeGridNodeDirection((unsigned int)nodeIdx);
        for (size_t i=0; i<groundOrigins.size(); i++)
        {
            if (gridSegmentContainsGridPoint(node, escape, groundOrigins[i]))
                return true;
        }
    }
    return false;
}

static bool placementRectsOverlap(const svPlacementRect& a,
                                  const svPlacementRect& b)
{
    return a.left <= b.right && a.right >= b.left &&
           a.top <= b.bottom && a.bottom >= b.top;
}

static bool placementConflicts(size_t deviceIdx,
                               const svBaseDeviceArray& devices,
                               const std::vector<bool>& placed)
{
    const svPlacementRect candidate = getPlacementRect(devices[deviceIdx]);
    for (size_t i=0; i<devices.size(); i++)
    {
        if (i == deviceIdx || !placed[i])
            continue;
        if (placementRectsOverlap(candidate, getPlacementRect(devices[i])))
            return true;
        if (terminalLeadHitsDeviceBody(devices[deviceIdx], devices[i]) ||
            terminalLeadHitsDeviceBody(devices[i], devices[deviceIdx]))
            return true;
        if (terminalLeadHitsGroundOrigin(devices[deviceIdx], devices[i]) ||
            terminalLeadHitsGroundOrigin(devices[i], devices[deviceIdx]))
            return true;
    }
    return false;
}

static bool getOccupiedPlacementBounds(const svBaseDeviceArray& devices,
                                       const std::vector<bool>& placed,
                                       svPlacementRect* bounds)
{
    bool haveBounds = false;
    for (size_t i=0; i<devices.size(); i++)
    {
        if (!placed[i])
            continue;

        const svPlacementRect rect = getPlacementRect(devices[i]);
        if (!haveBounds)
        {
            *bounds = rect;
            haveBounds = true;
        }
        else
        {
            bounds->left = std::min(bounds->left, rect.left);
            bounds->right = std::max(bounds->right, rect.right);
            bounds->top = std::min(bounds->top, rect.top);
            bounds->bottom = std::max(bounds->bottom, rect.bottom);
        }
    }
    return haveBounds;
}

static void choosePreferredRotation(svBaseDevice* dev)
{
    const char kind = dev->getSPICEid();
    if (kind == 'Q' || kind == 'M' || kind == 'J')
    {
        dev->setRotation(SVR_0);
        return;
    }

    if (dev->getNodesCount() != 2)
        return;

    const bool firstGround = dev->getNode(0) == svGroundNode;
    const bool secondGround = dev->getNode(1) == svGroundNode;
    const bool firstPower = isPowerNodeName(dev->getNode(0));
    const bool secondPower = isPowerNodeName(dev->getNode(1));

    if (firstGround)
        dev->setRotation(SVR_180);
    else if (secondGround || firstPower)
        dev->setRotation(SVR_0);
    else if (secondPower)
        dev->setRotation(SVR_180);
    else
        dev->setRotation(SVR_270);
}

static svGridEdge makeGridEdge(const wxPoint& first, const wxPoint& second)
{
    svGridPointLess less;
    svGridEdge edge;
    if (less(second, first))
    {
        edge.a = second;
        edge.b = first;
    }
    else
    {
        edge.a = first;
        edge.b = second;
    }
    return edge;
}

static void addGridSegmentEdges(svGridEdgeSet& edges,
                                const wxPoint& from, const wxPoint& to)
{
    if (from == to)
        return;

    wxASSERT(from.x == to.x || from.y == to.y);
    if (from.x != to.x && from.y != to.y)
        return;

    const wxPoint step(from.x == to.x ? 0 : (to.x > from.x ? 1 : -1),
                       from.y == to.y ? 0 : (to.y > from.y ? 1 : -1));
    wxPoint current = from;
    while (current != to)
    {
        const wxPoint next = current + step;
        edges.insert(makeGridEdge(current, next));
        current = next;
    }
}

static bool pointOnGridSegment(const wxPoint& point,
                               const wxPoint& from,
                               const wxPoint& to)
{
    if (from.x == to.x)
        return point.x == from.x &&
               point.y >= std::min(from.y, to.y) &&
               point.y <= std::max(from.y, to.y);
    if (from.y == to.y)
        return point.y == from.y &&
               point.x >= std::min(from.x, to.x) &&
               point.x <= std::max(from.x, to.x);
    return false;
}

static bool gridSegmentIntersectsRect(const wxPoint& from,
                                      const wxPoint& to,
                                      const wxRect& rect,
                                      unsigned int gridSize)
{
    if (from == to)
        return false;

    const int x1 = from.x * (int)gridSize;
    const int y1 = from.y * (int)gridSize;
    const int x2 = to.x * (int)gridSize;
    const int y2 = to.y * (int)gridSize;

    if (x1 == x2)
    {
        const int segTop = std::min(y1, y2);
        const int segBottom = std::max(y1, y2);
        return x1 >= rect.GetLeft() && x1 <= rect.GetRight() &&
               segBottom >= rect.GetTop() && segTop <= rect.GetBottom();
    }

    if (y1 == y2)
    {
        const int segLeft = std::min(x1, x2);
        const int segRight = std::max(x1, x2);
        return y1 >= rect.GetTop() && y1 <= rect.GetBottom() &&
               segRight >= rect.GetLeft() && segLeft <= rect.GetRight();
    }

    return false;
}

static bool routingSegmentBlocked(const wxPoint& from,
                                  const wxPoint& to,
                                  const svGridPointSet& blocked,
                                  const svRoutingObstacleArray& obstacles,
                                  unsigned int gridSize)
{
    for (svGridPointSet::const_iterator it=blocked.begin(); it!=blocked.end(); ++it)
    {
        if (pointOnGridSegment(*it, from, to))
            return true;
    }

    for (size_t i=0; i<obstacles.size(); i++)
    {
        if (gridSegmentIntersectsRect(from, to, obstacles[i].bounds, gridSize))
            return true;
    }

    return false;
}

static long blockedSegmentPenalty(const wxPoint& from,
                                  const wxPoint& to,
                                  const svGridPointSet& blocked,
                                  const svRoutingObstacleArray& obstacles,
                                  unsigned int gridSize)
{
    long penalty = 0;
    for (svGridPointSet::const_iterator it=blocked.begin(); it!=blocked.end(); ++it)
        if (pointOnGridSegment(*it, from, to))
            penalty += 10000;

    for (size_t i=0; i<obstacles.size(); i++)
    {
        if (gridSegmentIntersectsRect(from, to, obstacles[i].bounds, gridSize))
            penalty += 100000;
    }

    return penalty;
}

static void addRoutingLaneCandidates(std::set<int>& xs,
                                     std::set<int>& ys,
                                     const svRoutingObstacleArray& obstacles,
                                     unsigned int gridSize)
{
    for (size_t i=0; i<obstacles.size(); i++)
    {
        const wxRect& r = obstacles[i].bounds;
        const int left = (int)std::floor((double)r.GetLeft() / gridSize) - 1;
        const int right = (int)std::ceil((double)r.GetRight() / gridSize) + 1;
        const int top = (int)std::floor((double)r.GetTop() / gridSize) - 1;
        const int bottom = (int)std::ceil((double)r.GetBottom() / gridSize) + 1;
        xs.insert(left);
        xs.insert(right);
        ys.insert(top);
        ys.insert(bottom);
    }
}

static bool findGridFallbackRoute(const wxPoint& from,
                                  const wxPoint& fromDirection,
                                  const wxPoint& to,
                                  const wxPoint& toDirection,
                                  bool constrainFromDirection,
                                  bool constrainToDirection,
                                  const svGridPointSet& blocked,
                                  const svRoutingObstacleArray& obstacles,
                                  unsigned int gridSize,
                                  std::vector<wxPoint>* route)
{
    int minX = std::min(from.x, to.x);
    int maxX = std::max(from.x, to.x);
    int minY = std::min(from.y, to.y);
    int maxY = std::max(from.y, to.y);

    for (size_t i=0; i<obstacles.size(); i++)
    {
        const wxRect& r = obstacles[i].bounds;
        minX = std::min(minX,
            (int)std::floor((double)r.GetLeft() / gridSize) - 2);
        maxX = std::max(maxX,
            (int)std::ceil((double)r.GetRight() / gridSize) + 2);
        minY = std::min(minY,
            (int)std::floor((double)r.GetTop() / gridSize) - 2);
        maxY = std::max(maxY,
            (int)std::ceil((double)r.GetBottom() / gridSize) + 2);
    }

    minX -= 4;
    maxX += 4;
    minY -= 4;
    maxY += 4;

    std::queue<wxPoint> pending;
    std::map<wxPoint, wxPoint, svGridPointLess> parent;
    std::set<wxPoint, svGridPointLess> visited;
    pending.push(from);
    visited.insert(from);

    const wxPoint directions[4] = {
        wxPoint(1, 0), wxPoint(-1, 0),
        wxPoint(0, 1), wxPoint(0, -1)
    };

    bool found = from == to;
    while (!pending.empty() && !found)
    {
        const wxPoint current = pending.front();
        pending.pop();

        for (size_t i=0; i<4; i++)
        {
            const wxPoint direction = directions[i];
            if (current == from && constrainFromDirection &&
                direction.x*fromDirection.x + direction.y*fromDirection.y < 0)
                continue;

            const wxPoint next = current + direction;
            if (next.x < minX || next.x > maxX ||
                next.y < minY || next.y > maxY)
                continue;

            if (next == to && constrainToDirection &&
                direction.x*toDirection.x + direction.y*toDirection.y > 0)
                continue;

            if (visited.find(next) != visited.end())
                continue;
            if (routingSegmentBlocked(
                    current, next, blocked, obstacles, gridSize))
                continue;

            visited.insert(next);
            parent[next] = current;
            if (next == to)
            {
                found = true;
                break;
            }
            pending.push(next);
        }
    }

    if (!found)
    {
        wxLogWarning(
            "No obstacle-free grid route from (%d,%d) to (%d,%d), bounds x=%d..%d y=%d..%d",
            from.x, from.y, to.x, to.y, minX, maxX, minY, maxY);
        const wxPoint fromPixel = from * gridSize;
        const wxPoint toPixel = to * gridSize;
        for (size_t i=0; i<obstacles.size(); i++)
        {
            if (obstacles[i].bounds.Contains(fromPixel))
            {
                wxLogWarning(
                    "  from point is inside routing obstacle %u (%s), bbox=(%d,%d)-(%d,%d), gridpos=(%d,%d)",
                    (unsigned)i,
                    obstacles[i].device
                        ? obstacles[i].device->getName().c_str()
                        : "?",
                    obstacles[i].bounds.GetLeft(),
                    obstacles[i].bounds.GetTop(),
                    obstacles[i].bounds.GetRight(),
                    obstacles[i].bounds.GetBottom(),
                    obstacles[i].device
                        ? obstacles[i].device->getGridPosition().x : 0,
                    obstacles[i].device
                        ? obstacles[i].device->getGridPosition().y : 0);
                if (obstacles[i].device)
                {
                    for (size_t nodeIdx=0;
                         nodeIdx<obstacles[i].device->getNodesCount();
                         nodeIdx++)
                    {
                        const wxPoint node =
                            obstacles[i].device->getGridPosition() +
                            obstacles[i].device->getRelativeGridNodePosition(
                                (unsigned int)nodeIdx);
                        const wxPoint dir =
                            obstacles[i].device->getRelativeGridNodeDirection(
                                (unsigned int)nodeIdx);
                        wxLogWarning(
                            "    node %u %s at (%d,%d), dir=(%d,%d), escape=(%d,%d)",
                            (unsigned)nodeIdx,
                            obstacles[i].device->getNode((unsigned int)nodeIdx).c_str(),
                            node.x, node.y, dir.x, dir.y,
                            node.x + dir.x, node.y + dir.y);
                    }
                }
            }
            if (obstacles[i].bounds.Contains(toPixel))
                wxLogWarning(
                    "  to point is inside routing obstacle %u (%s)",
                    (unsigned)i,
                    obstacles[i].device
                        ? obstacles[i].device->getName().c_str()
                        : "?");
        }
        return false;
    }

    std::vector<wxPoint> reversed;
    wxPoint current = to;
    reversed.push_back(current);
    while (current != from)
    {
        std::map<wxPoint, wxPoint, svGridPointLess>::const_iterator it =
            parent.find(current);
        if (it == parent.end())
            return false;
        current = it->second;
        reversed.push_back(current);
    }
    std::reverse(reversed.begin(), reversed.end());

    std::vector<wxPoint> compressed;
    for (size_t i=0; i<reversed.size(); i++)
    {
        if (compressed.size() < 2)
        {
            compressed.push_back(reversed[i]);
            continue;
        }

        const wxPoint prevDelta =
            compressed[compressed.size()-1] -
            compressed[compressed.size()-2];
        const wxPoint nextDelta =
            reversed[i] - compressed[compressed.size()-1];
        const bool sameAxis =
            (prevDelta.x == 0 && nextDelta.x == 0) ||
            (prevDelta.y == 0 && nextDelta.y == 0);
        if (sameAxis)
            compressed.back() = reversed[i];
        else
            compressed.push_back(reversed[i]);
    }

    *route = compressed;
    return true;
}

static bool findObstacleAwareRoute(const wxPoint& from,
                                   const wxPoint& fromDirection,
                                   const wxPoint& to,
                                   const wxPoint& toDirection,
                                   bool constrainFromDirection,
                                   bool constrainToDirection,
                                   const svGridPointSet& blocked,
                                   const svRoutingObstacleArray& obstacles,
                                   unsigned int gridSize,
                                   std::vector<wxPoint>* route)
{
    struct Candidate
    {
        std::vector<wxPoint> points;
        long score;
    };

    Candidate best;
    best.score = LONG_MAX;

    const auto consider = [&](const std::vector<wxPoint>& rawPoints)
    {
        std::vector<wxPoint> points;
        for (size_t i=0; i<rawPoints.size(); i++)
        {
            if (points.empty() || points.back() != rawPoints[i])
                points.push_back(rawPoints[i]);
        }
        if (points.size() < 2)
            return;

        size_t firstSegment = 0;
        while (firstSegment + 1 < points.size() &&
               points[firstSegment] == points[firstSegment+1])
            firstSegment++;
        size_t lastSegment = points.size() - 1;
        while (lastSegment > 0 &&
               points[lastSegment] == points[lastSegment-1])
            lastSegment--;

        if (firstSegment + 1 >= points.size() || lastSegment == 0)
            return;

        const wxPoint firstDelta =
            points[firstSegment+1] - points[firstSegment];
        if (constrainFromDirection &&
            firstDelta.x*fromDirection.x + firstDelta.y*fromDirection.y < 0)
            return;

        const wxPoint lastDelta =
            points[lastSegment] - points[lastSegment-1];
        if (constrainToDirection &&
            lastDelta.x*toDirection.x + lastDelta.y*toDirection.y > 0)
            return;

        long score = 0;
        int bends = 0;
        wxPoint previousDelta(0, 0);
        for (size_t i=1; i<points.size(); i++)
        {
            const wxPoint delta = points[i] - points[i-1];
            if (delta == wxPoint(0, 0))
                continue;
            if (delta.x != 0 && delta.y != 0)
                return;
            if (routingSegmentBlocked(
                    points[i-1], points[i], blocked, obstacles, gridSize))
                return;

            score += std::abs(delta.x) + std::abs(delta.y);
            if (previousDelta != wxPoint(0, 0) &&
                ((previousDelta.x == 0) != (delta.x == 0)))
                bends++;
            previousDelta = delta;
        }
        score += bends * 2;

        if (score < best.score)
        {
            best.points = points;
            best.score = score;
        }
    };

    if (from.x == to.x || from.y == to.y)
        consider(std::vector<wxPoint>{from, to});

    consider(std::vector<wxPoint>{
        from, wxPoint(to.x, from.y), to});
    consider(std::vector<wxPoint>{
        from, wxPoint(from.x, to.y), to});

    const int minX = std::min(from.x, to.x);
    const int maxX = std::max(from.x, to.x);
    const int minY = std::min(from.y, to.y);
    const int maxY = std::max(from.y, to.y);
    std::set<int> xs;
    std::set<int> ys;
    for (int x=minX-8; x<=maxX+8; x++)
        xs.insert(x);
    for (int y=minY-8; y<=maxY+8; y++)
        ys.insert(y);
    addRoutingLaneCandidates(xs, ys, obstacles, gridSize);

    for (std::set<int>::const_iterator it=xs.begin(); it!=xs.end(); ++it)
    {
        const int x = *it;
        consider(std::vector<wxPoint>{
            from,
            wxPoint(x, from.y),
            wxPoint(x, to.y),
            to});
    }
    for (std::set<int>::const_iterator it=ys.begin(); it!=ys.end(); ++it)
    {
        const int y = *it;
        consider(std::vector<wxPoint>{
            from,
            wxPoint(from.x, y),
            wxPoint(to.x, y),
            to});
    }

    // A single shared corridor handles almost every schematic route. If a
    // dense arrangement blocks all such candidates, try an outer rectangular
    // detour using the extreme clear lanes gathered above. This keeps failure
    // from silently disconnecting a net in crowded circuits.
    if (best.score == LONG_MAX && !xs.empty() && !ys.empty())
    {
        const int outerX[2] = {*xs.begin(), *xs.rbegin()};
        const int outerY[2] = {*ys.begin(), *ys.rbegin()};
        for (size_t xi=0; xi<2; xi++)
        {
            for (size_t yi=0; yi<2; yi++)
            {
                const int x = outerX[xi];
                const int y = outerY[yi];
                consider(std::vector<wxPoint>{
                    from,
                    wxPoint(x, from.y),
                    wxPoint(x, y),
                    wxPoint(to.x, y),
                    to});
                consider(std::vector<wxPoint>{
                    from,
                    wxPoint(from.x, y),
                    wxPoint(x, y),
                    wxPoint(x, to.y),
                    to});
            }
        }
    }

    if (best.score == LONG_MAX)
    {
        return findGridFallbackRoute(
            from, fromDirection, to, toDirection,
            constrainFromDirection, constrainToDirection,
            blocked, obstacles, gridSize, route);
    }

    *route = best.points;
    return true;
}

static void addRouteEdges(svGridEdgeSet& edges,
                          const std::vector<wxPoint>& route)
{
    for (size_t i=1; i<route.size(); i++)
        addGridSegmentEdges(edges, route[i-1], route[i]);
}

static wxPoint trimRouteAtSharedTrunk(std::vector<wxPoint>* route,
                                      bool horizontal,
                                      int trunkCoordinate)
{
    if (route->empty())
        return wxPoint(0, 0);

    for (size_t i=0; i<route->size(); i++)
    {
        const wxPoint current = (*route)[i];
        const bool currentOnTrunk = horizontal
            ? current.y == trunkCoordinate
            : current.x == trunkCoordinate;
        if (currentOnTrunk)
        {
            route->resize(i + 1);
            return current;
        }

        if (i + 1 >= route->size())
            break;

        const wxPoint next = (*route)[i + 1];
        wxPoint intersection;
        bool crossesTrunk = false;
        if (horizontal && current.x == next.x)
        {
            crossesTrunk =
                trunkCoordinate >= std::min(current.y, next.y) &&
                trunkCoordinate <= std::max(current.y, next.y);
            intersection = wxPoint(current.x, trunkCoordinate);
        }
        else if (!horizontal && current.y == next.y)
        {
            crossesTrunk =
                trunkCoordinate >= std::min(current.x, next.x) &&
                trunkCoordinate <= std::max(current.x, next.x);
            intersection = wxPoint(trunkCoordinate, current.y);
        }

        if (crossesTrunk)
        {
            route->resize(i + 1);
            if (route->empty() || route->back() != intersection)
                route->push_back(intersection);
            return intersection;
        }
    }

    return route->back();
}

static void pruneUnterminatedGridLeaves(
    svGridEdgeSet* edges,
    const std::vector<svWireTerminal>& terminals)
{
    svGridPointSet realTerminals;
    for (size_t i=0; i<terminals.size(); i++)
        realTerminals.insert(terminals[i].node);

    while (!edges->empty())
    {
        std::map<wxPoint, unsigned int, svGridPointLess> degree;
        for (svGridEdgeSet::const_iterator it=edges->begin();
             it!=edges->end(); ++it)
        {
            degree[it->a]++;
            degree[it->b]++;
        }

        std::vector<svGridEdge> toRemove;
        for (svGridEdgeSet::const_iterator it=edges->begin();
             it!=edges->end(); ++it)
        {
            const bool aIsFloatingLeaf =
                degree[it->a] == 1 &&
                realTerminals.find(it->a) == realTerminals.end();
            const bool bIsFloatingLeaf =
                degree[it->b] == 1 &&
                realTerminals.find(it->b) == realTerminals.end();
            if (aIsFloatingLeaf || bIsFloatingLeaf)
                toRemove.push_back(*it);
        }

        if (toRemove.empty())
            break;
        for (size_t i=0; i<toRemove.size(); i++)
            edges->erase(toRemove[i]);
    }
}

static long scoreHorizontalTrunk(const std::vector<svWireTerminal>& terminals,
                                 int trunkY, int minX, int maxX,
                                 const svGridPointSet& blocked,
                                 const svRoutingObstacleArray& obstacles,
                                 unsigned int gridSize)
{
    long score = maxX - minX;
    if (routingSegmentBlocked(
            wxPoint(minX, trunkY), wxPoint(maxX, trunkY),
            blocked, obstacles, gridSize))
        return LONG_MAX;

    for (size_t i=0; i<terminals.size(); i++)
    {
        const int delta = trunkY - terminals[i].escape.y;
        score += std::abs(delta);
        score += blockedSegmentPenalty(
            terminals[i].escape,
            wxPoint(terminals[i].escape.x, trunkY), blocked,
            obstacles, gridSize);

        if (terminals[i].direction.x != 0 && delta != 0)
            score += 2;
        else if (terminals[i].direction.y != 0 &&
                 delta * terminals[i].direction.y < 0)
            score += 6 + 2*std::abs(delta);
    }
    return score;
}

static long scoreVerticalTrunk(const std::vector<svWireTerminal>& terminals,
                               int trunkX, int minY, int maxY,
                               const svGridPointSet& blocked,
                               const svRoutingObstacleArray& obstacles,
                               unsigned int gridSize)
{
    long score = maxY - minY;
    if (routingSegmentBlocked(
            wxPoint(trunkX, minY), wxPoint(trunkX, maxY),
            blocked, obstacles, gridSize))
        return LONG_MAX;

    for (size_t i=0; i<terminals.size(); i++)
    {
        const int delta = trunkX - terminals[i].escape.x;
        score += std::abs(delta);
        score += blockedSegmentPenalty(
            terminals[i].escape,
            wxPoint(trunkX, terminals[i].escape.y), blocked,
            obstacles, gridSize);

        if (terminals[i].direction.y != 0 && delta != 0)
            score += 2;
        else if (terminals[i].direction.x != 0 &&
                 delta * terminals[i].direction.x < 0)
            score += 6 + 2*std::abs(delta);
    }
    return score;
}

static bool chooseHorizontalSharedTrunk(
    const std::vector<svWireTerminal>& terminals,
    const svGridPointSet& blocked,
    const svRoutingObstacleArray& obstacles,
    unsigned int gridSize,
    int* coordinate)
{
    int minX = terminals[0].escape.x;
    int maxX = terminals[0].escape.x;
    int minY = terminals[0].escape.y;
    int maxY = terminals[0].escape.y;
    for (size_t i=1; i<terminals.size(); i++)
    {
        minX = std::min(minX, terminals[i].escape.x);
        maxX = std::max(maxX, terminals[i].escape.x);
        minY = std::min(minY, terminals[i].escape.y);
        maxY = std::max(maxY, terminals[i].escape.y);
    }

    std::set<int> candidateXs;
    std::set<int> candidateYs;
    for (int x=minX-8; x<=maxX+8; x++)
        candidateXs.insert(x);
    for (int y=minY-8; y<=maxY+8; y++)
        candidateYs.insert(y);
    addRoutingLaneCandidates(
        candidateXs, candidateYs, obstacles, gridSize);

    long bestHorizontalScore = LONG_MAX;
    int bestHorizontalY = minY;
    for (std::set<int>::const_iterator it=candidateYs.begin();
         it!=candidateYs.end(); ++it)
    {
        const int y = *it;
        const long score =
            scoreHorizontalTrunk(terminals, y, minX, maxX, blocked,
                                 obstacles, gridSize);
        if (score < bestHorizontalScore)
        {
            bestHorizontalScore = score;
            bestHorizontalY = y;
        }
    }

    long bestVerticalScore = LONG_MAX;
    int bestVerticalX = minX;
    for (std::set<int>::const_iterator it=candidateXs.begin();
         it!=candidateXs.end(); ++it)
    {
        const int x = *it;
        const long score =
            scoreVerticalTrunk(terminals, x, minY, maxY, blocked,
                               obstacles, gridSize);
        if (score < bestVerticalScore)
        {
            bestVerticalScore = score;
            bestVerticalX = x;
        }
    }

    // With finite component obstacles and the extra outer lanes above, at
    // least one straight shared trunk should normally exist. Keep a stable
    // fallback outside every obstacle/ground marker if an extreme malformed
    // placement defeats that assumption; branch routing below remains
    // obstacle-aware.
    if (bestHorizontalScore == LONG_MAX && bestVerticalScore == LONG_MAX)
    {
        int outerY = minY - 4;
        for (size_t i=0; i<obstacles.size(); i++)
        {
            const int obstacleTop =
                (int)std::floor(
                    (double)obstacles[i].bounds.GetTop() / gridSize);
            outerY = std::min(outerY, obstacleTop - 2);
        }
        for (svGridPointSet::const_iterator it=blocked.begin();
             it!=blocked.end(); ++it)
        {
            outerY = std::min(outerY, it->y - 2);
        }
        *coordinate = outerY;
        return true;
    }

    const bool horizontal =
        bestVerticalScore == LONG_MAX ||
        bestHorizontalScore < bestVerticalScore ||
        (bestHorizontalScore == bestVerticalScore &&
         (maxX-minX) >= (maxY-minY));
    *coordinate = horizontal ? bestHorizontalY : bestVerticalX;
    return horizontal;
}

static void drawSharedManhattanTree(wxGraphicsContext* gc,
                                    const std::vector<svWireTerminal>& terminals,
                                    const svGridPointSet& blocked,
                                    const svRoutingObstacleArray& obstacles,
                                    unsigned int gridSize)
{
    if (terminals.empty())
        return;

    svGridEdgeSet edges;
    for (size_t i=0; i<terminals.size(); i++)
        addGridSegmentEdges(edges, terminals[i].node, terminals[i].escape);

    int trunkCoordinate = 0;
    const bool horizontal =
        chooseHorizontalSharedTrunk(
            terminals, blocked, obstacles, gridSize, &trunkCoordinate);

    int minAlong = INT_MAX;
    int maxAlong = INT_MIN;
    for (size_t i=0; i<terminals.size(); i++)
    {
        const wxPoint branch = horizontal
            ? wxPoint(terminals[i].escape.x, trunkCoordinate)
            : wxPoint(trunkCoordinate, terminals[i].escape.y);

        std::vector<wxPoint> route;
        bool routed = findObstacleAwareRoute(
                terminals[i].escape,
                terminals[i].direction,
                branch,
                wxPoint(0, 0),
                true,
                false,
                blocked,
                obstacles,
                gridSize,
                &route);
        if (!routed)
        {
            routed = findObstacleAwareRoute(
                terminals[i].escape,
                wxPoint(0, 0),
                branch,
                wxPoint(0, 0),
                false,
                false,
                blocked,
                obstacles,
                gridSize,
                &route);
        }

        if (routed)
        {
            const wxPoint attachment =
                trimRouteAtSharedTrunk(&route, horizontal, trunkCoordinate);
            addRouteEdges(edges, route);
            const int along = horizontal ? attachment.x : attachment.y;
            minAlong = std::min(minAlong, along);
            maxAlong = std::max(maxAlong, along);
        }
        else
        {
            wxLogError(
                "Unable to find an obstacle-free shared-net branch route");
            continue;
        }
    }

    if (minAlong <= maxAlong)
    {
        const wxPoint trunkStart = horizontal
            ? wxPoint(minAlong, trunkCoordinate)
            : wxPoint(trunkCoordinate, minAlong);
        const wxPoint trunkEnd = horizontal
            ? wxPoint(maxAlong, trunkCoordinate)
            : wxPoint(trunkCoordinate, maxAlong);
        addGridSegmentEdges(edges, trunkStart, trunkEnd);
    }

    // The branch router can touch a shared trunk before its nominal projected
    // target. Never leave the unused remainder as a degree-one wire tail.
    // Recursively remove every leaf that is not an actual device terminal so
    // the rendered net cannot end in empty space.
    pruneUnterminatedGridLeaves(&edges, terminals);

    // Unit grid edges are deduplicated in the set, so overlapping branch and
    // trunk sections are stroked once even when several terminals share them.
    for (svGridEdgeSet::const_iterator it=edges.begin(); it!=edges.end(); ++it)
        drawGridLine(gc, it->a, it->b, gridSize);

    std::map<wxPoint, unsigned int, svGridPointLess> degree;
    for (svGridEdgeSet::const_iterator it=edges.begin(); it!=edges.end(); ++it)
    {
        degree[it->a]++;
        degree[it->b]++;
    }

    gc->SetBrush(*wxBLACK_BRUSH);
    const double r = std::max(2.5, gridSize / 12.0);
    for (std::map<wxPoint, unsigned int, svGridPointLess>::const_iterator
         it=degree.begin(); it!=degree.end(); ++it)
    {
        if (it->second < 3)
            continue;

        gc->DrawEllipse(it->first.x*gridSize-r, it->first.y*gridSize-r,
                        2*r, 2*r);
    }
}

static void drawPowerNetMarker(wxGraphicsContext* gc,
                               const svWireTerminal& terminal,
                               const svNode& node,
                               unsigned int gridSize)
{
    drawGridLine(gc, terminal.node, terminal.escape, gridSize);

    const wxRealPoint p = terminal.escape * gridSize;
    const wxRealPoint dir(terminal.direction.x, terminal.direction.y);
    const wxRealPoint perpendicular(-terminal.direction.y, terminal.direction.x);
    const double stemLength = gridSize * 0.34;
    const double wingOffset = gridSize * 0.18;
    const double wingBack = gridSize * 0.13;

    const wxRealPoint apex(p.x + dir.x*stemLength,
                           p.y + dir.y*stemLength);
    const wxRealPoint wingCenter(p.x + dir.x*wingBack,
                                 p.y + dir.y*wingBack);
    const wxRealPoint wing1(wingCenter.x + perpendicular.x*wingOffset,
                            wingCenter.y + perpendicular.y*wingOffset);
    const wxRealPoint wing2(wingCenter.x - perpendicular.x*wingOffset,
                            wingCenter.y - perpendicular.y*wingOffset);

    drawLine(gc, p, apex);
    drawLine(gc, apex, wing1);
    drawLine(gc, apex, wing2);

    const wxString label = wxString::FromUTF8(node.c_str()).Upper();
    wxDouble textW = 0, textH = 0, descent = 0, leading = 0;
    gc->GetTextExtent(label, &textW, &textH, &descent, &leading);

    double x = apex.x - textW/2.0;
    double y = apex.y - textH - 3;
    if (terminal.direction == wxPoint(0, 1))
        y = apex.y + 3;
    else if (terminal.direction == wxPoint(-1, 0))
    {
        x = apex.x - textW - 4;
        y = apex.y - textH/2.0;
    }
    else if (terminal.direction == wxPoint(1, 0))
    {
        x = apex.x + 4;
        y = apex.y - textH/2.0;
    }

    gc->DrawText(label, x, y);
}

static void drawUsefulSignalLabel(wxGraphicsContext* gc,
                                  const std::vector<svWireTerminal>& terminals,
                                  const svNode& node,
                                  unsigned int gridSize)
{
    if (!isUsefulSignalLabel(node) || terminals.empty())
        return;

    for (size_t i=0; i<terminals.size(); i++)
        if (terminals[i].externalPin)
            return;

    wxPoint anchor = terminals[0].escape;
    bool foundJunction = false;
    for (size_t i=0; i<terminals.size() && !foundJunction; i++)
    {
        for (size_t j=i+1; j<terminals.size(); j++)
        {
            if (terminals[i].escape == terminals[j].escape)
            {
                anchor = terminals[i].escape;
                foundJunction = true;
                break;
            }
        }
    }

    const wxString label = wxString::FromUTF8(node.c_str()).Upper();
    wxFont font = *wxSWISS_FONT;
    if (font.GetPointSize() > 7)
        font.SetPointSize(font.GetPointSize()-1);
    gc->SetFont(font, *wxBLACK);

    wxDouble textW = 0, textH = 0, descent = 0, leading = 0;
    gc->GetTextExtent(label, &textW, &textH, &descent, &leading);
    gc->DrawText(label,
                 anchor.x*gridSize + 6,
                 anchor.y*gridSize - textH - 4);
}

static void drawOrthogonalConnection(wxGraphicsContext* gc,
                                     const svWireTerminal& from,
                                     const svWireTerminal& to,
                                     const svGridPointSet& blocked,
                                     const svRoutingObstacleArray& obstacles,
                                     unsigned int gridSize)
{
    if (from.escape == to.escape)
        return;

    std::vector<wxPoint> route;
    bool routed = findObstacleAwareRoute(
            from.escape,
            from.direction,
            to.escape,
            to.direction,
            true,
            true,
            blocked,
            obstacles,
            gridSize,
            &route);
    if (!routed)
    {
        routed = findObstacleAwareRoute(
            from.escape,
            wxPoint(0, 0),
            to.escape,
            wxPoint(0, 0),
            false,
            false,
            blocked,
            obstacles,
            gridSize,
            &route);
    }

    if (!routed)
    {
        wxLogError(
            "Unable to find an obstacle-free two-terminal route");
        return;
    }

    for (size_t i=1; i<route.size(); i++)
        drawGridLine(gc, route[i-1], route[i], gridSize);
}

static svGridPointSet collectGroundRoutingBlocks(
    const svBaseDeviceArray& devices)
{
    svGridPointSet blocked;
    for (size_t deviceIdx=0; deviceIdx<devices.size(); deviceIdx++)
    {
        for (size_t nodeIdx=0; nodeIdx<devices[deviceIdx]->getNodesCount(); nodeIdx++)
        {
            if (devices[deviceIdx]->getNode((unsigned int)nodeIdx) != svGroundNode)
                continue;
            const wxPoint node =
                devices[deviceIdx]->getGridPosition() +
                devices[deviceIdx]->getRelativeGridNodePosition((unsigned int)nodeIdx);
            const wxPoint direction =
                devices[deviceIdx]->getRelativeGridNodeDirection((unsigned int)nodeIdx);
            blocked.insert(node + direction);
        }
    }
    return blocked;
}

static svRoutingObstacleArray collectDeviceRoutingObstacles(
    const svBaseDeviceArray& devices, unsigned int gridSize)
{
    svRoutingObstacleArray obstacles;
    const int margin = std::max(2, (int)gridSize / 12);

    for (size_t i=0; i<devices.size(); i++)
    {
        wxRect bounds = devices[i]->getRealBoundingBox(gridSize);
        if (bounds.GetWidth() <= 0 && bounds.GetHeight() <= 0)
            continue;

        bounds.Inflate(margin, margin);

        svRoutingObstacle obstacle;
        obstacle.bounds = bounds;
        obstacle.device = devices[i];
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

static double getGroundRotation(const wxPoint& direction)
{
    if (direction == wxPoint(-1, 0))
        return M_PI / 2;
    if (direction == wxPoint(0, -1))
        return M_PI;
    if (direction == wxPoint(1, 0))
        return 3 * M_PI / 2;
    return 0;
}

static bool containsDigit(const wxString& value)
{
    for (size_t i=0; i<value.size(); i++)
        if (value[i] >= '0' && value[i] <= '9')
            return true;
    return false;
}
}


/*
    For more informations about the SPICE netlist format please go to:
      http://www.ecircuitcenter.com/SPICEsummary.htm
*/


// ============================================================================
// implementation
// ============================================================================

// ----------------------------------------------------------------------------
// globals
// ----------------------------------------------------------------------------

wxPoint svInvalidPoint = wxPoint(-1e9, -1e9);
svNode svGroundNode = svNode("0");      // SPICE conventional name for GND

#define ALLOWED_CHARS       "0123456789.+-"

struct {
    const char* postfixShort;
    const char* postfixLong;
    double multiplier;
} g_mult[] = 
{
    { "F", "FEMTO", 1e-15 },
    { "P", "PICO", 1e-12 },
    { "N", "NANO", 1e-9 },
    { "U", "MICRO", 1e-6 },
    { "M", "MILLI", 1e-3 },
    { "K", "KILO", 1e3 },
    { "MEG", "MEGA", 1e6 },
    { "G", "GIGA", 1e9 },
    { "T", "TERA", 1e12 }
};

struct {
    const char* nameShort;
    const char* nameLong;
} g_units[] = 
{
    { "F", "FARAD" },
    { "OHM", "" },
    { "H", "HENRY" },
    { "A", "AMPERE" },
    { "V", "VOLT" }
};

// ----------------------------------------------------------------------------
// svString
// ----------------------------------------------------------------------------

extern std::string eng(double value, int digits, int numeric);


/* static */
svString svString::formatValue(double v)
{
    std::string formatted = eng(v, 3, 0);
    formatted.erase(std::remove(formatted.begin(), formatted.end(), ' '),
                    formatted.end());

    size_t suffix = 0;
    while (suffix < formatted.size() &&
           (std::isdigit((unsigned char)formatted[suffix]) ||
            formatted[suffix] == '-' || formatted[suffix] == '+' ||
            formatted[suffix] == '.'))
    {
        suffix++;
    }

    const size_t decimal = formatted.find('.');
    if (decimal != std::string::npos && decimal < suffix)
    {
        size_t end = suffix;
        while (end > decimal + 1 && formatted[end-1] == '0')
            end--;
        if (end == decimal + 1)
            end = decimal;
        formatted.erase(end, suffix-end);
    }

    return svString(formatted);
}

bool svString::startsWithOneOf(const std::string& str, unsigned int* len) const
{
    if (len) 
        *len = 0;

    for (size_t i=0; i<this->size(); i++)
    {
        bool ok = false;
        for (size_t j=0; j<str.size(); j++)
        {
            if (at(i) == str.at(j))
            {
                ok = true;
                break;
            }
        }

        if (i == 0 && !len)
            return ok;
        else if (ok)
            (*len)++;
        else if (!ok)
            break;
    }

    return true;
}

bool svString::getValue(double *res) const
{
    unsigned int numlen;
    if (!startsWithOneOf(ALLOWED_CHARS, &numlen))
    {
        *res = 0;
        return false;
    }

    double firstPart = atof(substr(0, numlen).c_str());
    std::string secondPartStr;
    for (size_t i=numlen; i<size(); i++)
        secondPartStr.push_back(toupper(at(i)));

    if (secondPartStr.size() == 0)
    {
        *res = firstPart;
        return true;
    }

    std::string thirdPartStr;
    double secondPart;
    if (secondPartStr.at(0) == 'E')
    {
        if (!svString(secondPartStr.substr(1)).startsWithOneOf(ALLOWED_CHARS, &numlen))
        {
            *res = 0;
            return false;
        }
        
        int exp = atoi(secondPartStr.substr(1, 1+numlen).c_str());
        secondPart = pow(10.0, exp);

        thirdPartStr = secondPartStr.substr(1+numlen);
    }
    else
    {
        secondPart = 0;
        for (size_t i=0; i<WXSIZEOF(g_mult); i++)
        {
            if (wxString(secondPartStr).StartsWith(g_mult[i].postfixLong))
            {
                secondPart = g_mult[i].multiplier;
                thirdPartStr = secondPartStr.substr(strlen(g_mult[i].postfixLong));
                break;
            }
        }

        // try searching for the "short" postfix
        if (secondPart == 0)
        {
            for (size_t i=0; i<WXSIZEOF(g_mult); i++)
            {
                if (wxString(secondPartStr).StartsWith(g_mult[i].postfixShort))
                {
                    secondPart = g_mult[i].multiplier;
                    thirdPartStr = secondPartStr.substr(strlen(g_mult[i].postfixShort));
                    break;
                }
            }
        }

        if (secondPart == 0)
        {
            // maybe there's no multiplier (e.g. "10Volt")
            secondPart = 1.0;
            thirdPartStr = secondPartStr;
        }
    }

    if (thirdPartStr.size() > 0)
    {
        // now we only need to parse 'thirdPartStr'...
        std::string fourthPartStr;
        for (size_t i=0; i<WXSIZEOF(g_units); i++)
        {
            if (strlen(g_units[i].nameLong) > 0 &&
                wxString(thirdPartStr).StartsWith(g_units[i].nameLong))
            {
                fourthPartStr = thirdPartStr.substr(strlen(g_units[i].nameLong));
                break;
            }
            if (wxString(thirdPartStr).StartsWith(g_units[i].nameShort))
            {
                fourthPartStr = thirdPartStr.substr(strlen(g_units[i].nameShort));
                break;
            }
        }

        if (fourthPartStr.size() > 0)
            return false;       // there should be nothing more to parse!
    }

    // last, compose the parsed number:
    *res = firstPart * secondPart;

    return true;
}

// ----------------------------------------------------------------------------
// svParserSPICE
// ----------------------------------------------------------------------------

bool svParserSPICE::load(svCircuitArray& ret, const std::string& filename)
{
    ret.clear();

    wxFileInputStream input_stream(filename);
    if (!input_stream.IsOk())
    {
        wxLogError("Cannot open file '%s'.", filename);
        return false;
    }

    wxStringOutputStream netlist_contents;
    if (input_stream.Read(netlist_contents).GetLastError() != wxSTREAM_EOF)
    {
        wxLogError("Cannot read file '%s'.", filename);
        return false;
    }

    // first of all, split the file at newline boundaries
    wxArrayString lines = wxStringTokenize(netlist_contents.GetString(), "\n");
    wxArrayString toparse;
    bool sawFirstNonEmptyLine = false;
    bool firstNonEmptyLineWasComment = false;
    for (size_t i=0; i<lines.size(); i++)
    {
        // remove unwanted blanks from start/end of each line
        lines[i].Trim(false /* from left */);
        lines[i].Trim(true /* from right */);

        // discard empty lines
        if (lines[i].size() == 0)
            continue;

        if (!sawFirstNonEmptyLine)
        {
            sawFirstNonEmptyLine = true;
            firstNonEmptyLineWasComment = lines[i].StartsWith("*");
        }

        // discard comments
        if (lines[i].StartsWith("*"))
            continue;

        // + is the continuation character in SPICE syntax
        if (lines[i].StartsWith("+"))
        {
            if (toparse.size() == 0)
            {
                wxLogError("Found a SPICE continuation line without a preceding statement.");
                return false;
            }

            wxString continuation = lines[i].Mid(1);
            continuation.Trim(false);
            toparse.back() += " " + continuation;
        }
        else
            toparse.push_back(lines[i]);
    }

    // FIXME: the line numbers reported in wxLogError from now on will have a 
    // "wrong" number since we removed empty lines and comment lines...
    // this should be fixed adding a toparse => lines index map table

    struct SubcircuitBlock
    {
        size_t startIdx;
        size_t endIdx;
        wxArrayString header;
    };

    // Discover explicit .SUBCKT blocks and remember which source lines belong
    // to them without parsing the model bodies yet. A simulator deck can
    // contain a supported top-level circuit plus complex embedded model
    // subcircuits that this lightweight viewer does not understand. Those
    // models should not block rendering the top-level circuit.
    std::vector<SubcircuitBlock> subcircuitBlocks;
    std::vector<bool> subcircuitLine(toparse.size(), false);
    for (size_t i=0; i<toparse.size(); i++)
    {
        wxArrayString subcktHeader =
            wxStringTokenize(toparse[i], " \t", wxTOKEN_DEFAULT);
        if (subcktHeader.size() > 0 &&
            subcktHeader[0].CmpNoCase(".END") == 0)
        {
            break;
        }

        if (subcktHeader.size() > 0 &&
            subcktHeader[0].CmpNoCase(".SUBCKT") == 0)
        {
            size_t startIdx = i+1;

            // search for the end of this .SUBCKT
            int endIdx = -1;
            for (size_t j=startIdx; j<toparse.size(); j++)
            {
                wxArrayString arr =
                    wxStringTokenize(toparse[j], " \t", wxTOKEN_DEFAULT);
                if (arr.size() > 0 && arr[0].CmpNoCase(".ENDS") == 0)
                {
                    endIdx = j;
                    break;
                }
            }

            if (endIdx == -1)
            {
                wxLogError("Could not find the .ENDS statement for the .SUBCKT statement of line %d", startIdx);
                return false;
            }

            for (size_t j=i; j<=(size_t)endIdx; j++)
                subcircuitLine[j] = true;

            SubcircuitBlock block;
            block.startIdx = startIdx;
            block.endIdx = (size_t)endIdx;
            block.header = subcktHeader;
            subcircuitBlocks.push_back(block);
            i = (size_t)endIdx;
        }
    }

    // A normal simulator deck, including the netlists generated by Labby
    // PSpice blocks, is usually a top-level circuit ending in .END. Collect
    // only top-level drawable statements, ignoring embedded .SUBCKT models and
    // simulator directives such as .DC, .AC, .TRAN, .PRINT, and .MODEL.
    wxArrayString circuitLines;
    bool checkedTitleLine = false;
    bool sawTopLevelEnd = false;
    bool explicitTitleProvided = firstNonEmptyLineWasComment;

    for (size_t i=0; i<toparse.size(); i++)
    {
        if (subcircuitLine[i])
            continue;

        wxArrayString fields =
            wxStringTokenize(toparse[i], " \t", wxTOKEN_DEFAULT);
        if (fields.size() == 0)
            continue;

        const wxString firstTokenUpper = fields[0].Upper();
        if (firstTokenUpper == ".END")
        {
            sawTopLevelEnd = true;
            break;
        }
        if (firstTokenUpper == ".TITLE")
        {
            explicitTitleProvided = true;
            continue;
        }
        if (firstTokenUpper.StartsWith("."))
            continue;

        if (!checkedTitleLine)
        {
            checkedTitleLine = true;

            if (!explicitTitleProvided)
            {
                // The first non-comment line in classic SPICE may be a title.
                // Keep it only when it clearly looks like a device statement.
                // An explicit leading '*' title/comment or .TITLE directive
                // bypasses this ambiguity and makes the first element line a
                // real device unconditionally.
                svBaseDevice* probe =
                    svDeviceFactory::getDeviceMatchingIdentifier(fields[0].Upper()[0]);
                const wxString suffix = fields[0].Mid(1);
                const bool explicitElementName =
                    containsDigit(suffix) ||
                    (suffix.size() > 1 &&
                     suffix == suffix.Upper() &&
                     suffix != suffix.Lower());

                bool plausibleDevice = false;
                if (probe && fields.size() - 1 >= probe->getNodesCount())
                {
                    plausibleDevice = explicitElementName;
                    for (size_t j=1; !plausibleDevice && j<fields.size(); j++)
                        plausibleDevice = containsDigit(fields[j]);
                }

                // Do not silently eat a likely unsupported element as a title
                // either. X is the standard subcircuit-instance designator,
                // and names such as U1 should reach the normal "unknown
                // component" error instead of yielding an incomplete drawing.
                const bool likelyUnsupportedElement =
                    (!probe && explicitElementName) ||
                    (fields[0].Upper().StartsWith("X") && fields[0].size() > 1);

                delete probe;
                if (!plausibleDevice && !likelyUnsupportedElement)
                {
                    continue;
                }
            }
        }

        circuitLines.push_back(toparse[i]);
    }

    if (circuitLines.size() > 0 &&
        (subcircuitBlocks.empty() || sawTopLevelEnd))
    {
        svCircuit circuit;
        circuit.setName(
            wxFileName(wxString::FromUTF8(filename.c_str())).GetName().ToStdString());
        if (!circuit.parseSPICESubCkt(circuitLines, 0, circuitLines.size()))
            return false;

        // A mixed simulator deck uses .SUBCKT blocks as model definitions.
        // Prefer the actual top-level circuit so GUI/headless callers still
        // receive one render target instead of tripping the old multi-circuit
        // limitation.
        ret.push_back(circuit);
    }
    else
    {
        for (size_t i=0; i<subcircuitBlocks.size(); i++)
        {
            const SubcircuitBlock& block = subcircuitBlocks[i];
            svCircuit sub;
            if (!sub.parseSPICESubCkt(toparse, block.startIdx, block.endIdx))
                return false;

            if (block.header.size() > 1)
                sub.setName(block.header[1].ToStdString());

            for (size_t j=2; j<block.header.size(); j++)
                // convert to lowercase because SPICE is case insensitive
                sub.addExternalNode(block.header[j].Lower().ToStdString());

            ret.push_back(sub);
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
// svCircuit
// ----------------------------------------------------------------------------

void svCircuit::addExternalNode(const svNode& extNode)
{ 
    m_nodes.insert(extNode); 
    addDevice(new svExternalPin(extNode));
}

bool svCircuit::parseSPICESubCkt(const wxArrayString& lines, size_t startIdx, size_t endIdx)
{
    release();

    for (size_t i=startIdx; i<endIdx; i++)
    {
        wxArrayString arr = wxStringTokenize(lines[i], " \t", wxTOKEN_DEFAULT);
        if (arr.size() <= 1)
            continue;

        wxString comp_name = arr[0];
        arr.erase(arr.begin());

        // Simulator directives affect analysis rather than schematic
        // connectivity. They can appear inside PSpice subcircuits as well as
        // top-level decks, so skip them here instead of treating them as
        // component identifiers.
        if (comp_name.StartsWith("."))
            continue;

        // first letter of the component identifies it:
        svBaseDevice* dev = svDeviceFactory::getDeviceMatchingIdentifier(comp_name.Upper()[0]);
        if (!dev)
        {
            wxLogError("Unknown component type for '%s'\n", comp_name);
            return false;
        }

        dev->setName(comp_name.substr(1).ToStdString());

        if (arr.size() < dev->getNodesCount())
        {
            wxLogError("At line %d: device '%s' is missing one (or more) of the required nodes", 
                       i, dev->getHumanReadableDesc().c_str());
            return false;
        }

        for (size_t j=0; j<dev->getNodesCount(); j++)
        {
            // convert to lowercase because SPICE is case insensitive
            std::string nodeName = arr[0].Lower().ToStdString();

            // add this node both to the global circuit and to the current device...
            addNode(nodeName);
            dev->addNode(nodeName);

            arr.erase(arr.begin());     // this node has been processed; remove it
        }

        wxASSERT(dev->getNodes().size() == dev->getNodesCount());

        for (size_t j=0; j<arr.size(); j++)
        {
            // there are additional properties device-specific:
            if (!dev->parseSPICEProperty(j, arr[j].ToStdString()))
            {
                wxLogError("Error parsing argument '%s' of line %d: '%s'", arr[j], i, lines[i]);
                return false;
            }
        }

        addDevice(dev);
    }

    return true;
}

svUGraph svCircuit::buildGraph() const
{
    if (m_nodes.empty())
        return svUGraph(0);

    // first, convert the std::set containing the circuit's nodes to a std::vector
    // so that we can associate each circuit node with a number (the node's index) 
    std::vector<svNode> circuitNodes;
    for (std::set<svNode>::const_iterator i = m_nodes.begin(); i != m_nodes.end(); i++)
        circuitNodes.push_back(*i);

    wxASSERT(circuitNodes[0] == svGroundNode);
    svUGraph ug(circuitNodes.size()-1 /* the node 0 (GND) does not need to be part of the graph */);

    // now create an "edge" in the graph for each device
    for (size_t i=0; i<m_devices.size(); i++)
    {
        // all nodes of the same device should be placed nearby...
        const std::vector<svNode>& deviceNodes = m_devices[i]->getNodes();
        std::vector<int> deviceNodeIndexes;
        for (size_t j=0; j<deviceNodes.size(); j++)
        {
            if (deviceNodes[j] != svGroundNode)
            {
                // find this node in the circuit's vector of nodes
                std::vector<svNode>::const_iterator 
                    result = find(circuitNodes.begin(), circuitNodes.end(), deviceNodes[j]);
                wxASSERT(result != circuitNodes.end());
                
                deviceNodeIndexes.push_back(result - circuitNodes.begin() - 1);
            }
        }

        for (size_t j=0; j<deviceNodeIndexes.size(); j++)
            for (size_t k=0; k<deviceNodeIndexes.size(); k++)
                if (k != j)
                    add_edge(deviceNodeIndexes[j], deviceNodeIndexes[k], ug);
    }

    return ug;
}

const wxRect& svCircuit::placeDevices(svPlaceAlgorithm ag)
{
    m_bb = wxRect(0,0,0,0);

    if (m_devices.size() == 0)
        return m_bb;

    switch (ag)
    {
    case SVPA_PLACE_NON_OVERLAPPED:
        {
            for (size_t i=0; i<m_devices.size(); i++)
                choosePreferredRotation(m_devices[i]);

            std::vector<bool> placed(m_devices.size(), false);
            std::map<svNode, wxPoint> netAnchors;
            std::map<svNode, unsigned int> groundBranches;
            std::map<svNode, unsigned int> railBranches;
            std::map<svNode, unsigned int> signalBranches;

            // Use transistors as the visual spine of the schematic. Their
            // natural orientation already follows the usual collector/drain
            // up, base/gate left, emitter/source down convention.
            unsigned int transistorCount = 0;
            for (size_t i=0; i<m_devices.size(); i++)
            {
                const char kind = m_devices[i]->getSPICEid();
                if (kind != 'Q' && kind != 'M' && kind != 'J')
                    continue;

                m_devices[i]->setGridPosition(
                    wxPoint(10 + (int)transistorCount * 5, 4));
                placed[i] = true;
                transistorCount++;

                for (size_t nodeIdx=0; nodeIdx<m_devices[i]->getNodesCount(); nodeIdx++)
                {
                    const svNode& node = m_devices[i]->getNode(nodeIdx);
                    if (node != svGroundNode &&
                        netAnchors.find(node) == netAnchors.end())
                    {
                        netAnchors[node] = getTerminalEscape(m_devices[i], nodeIdx);
                    }
                }
            }

            // Circuits without transistors still need a real topology seed.
            // Prefer a grounded independent source, then a floating source,
            // then the first ordinary two-terminal device. This keeps bridge
            // and isolated-source circuits out of the fallback row.
            if (netAnchors.empty())
            {
                const wxPoint seed(4, 5);
                size_t seedDevice = m_devices.size();

                for (size_t i=0; i<m_devices.size(); i++)
                {
                    if (m_devices[i]->getNodesCount() != 2)
                        continue;
                    if (m_devices[i]->getSPICEid() != 'V' &&
                        m_devices[i]->getSPICEid() != 'I')
                        continue;

                    int nonGround = wxNOT_FOUND;
                    if (m_devices[i]->getNode(0) == svGroundNode)
                        nonGround = 1;
                    else if (m_devices[i]->getNode(1) == svGroundNode)
                        nonGround = 0;
                    if (nonGround == wxNOT_FOUND)
                        continue;

                    alignTerminalEscape(m_devices[i], nonGround, seed);
                    placed[i] = true;
                    netAnchors[m_devices[i]->getNode(nonGround)] = seed;
                    seedDevice = i;
                    break;
                }

                if (seedDevice == m_devices.size())
                {
                    for (size_t i=0; i<m_devices.size(); i++)
                    {
                        if (m_devices[i]->getNodesCount() != 2)
                            continue;
                        if (m_devices[i]->getSPICEid() != 'V' &&
                            m_devices[i]->getSPICEid() != 'I')
                            continue;
                        if (m_devices[i]->getNode(0) == svGroundNode ||
                            m_devices[i]->getNode(1) == svGroundNode)
                            continue;

                        m_devices[i]->setRotation(SVR_270);
                        alignTerminalEscape(m_devices[i], 0, seed);
                        placed[i] = true;
                        netAnchors[m_devices[i]->getNode(0)] =
                            getTerminalEscape(m_devices[i], 0);
                        netAnchors[m_devices[i]->getNode(1)] =
                            getTerminalEscape(m_devices[i], 1);
                        seedDevice = i;
                        break;
                    }
                }

                if (seedDevice == m_devices.size())
                {
                    for (size_t i=0; i<m_devices.size(); i++)
                    {
                        if (m_devices[i]->getNodesCount() != 2)
                            continue;

                        const bool firstGround =
                            m_devices[i]->getNode(0) == svGroundNode;
                        const bool secondGround =
                            m_devices[i]->getNode(1) == svGroundNode;
                        if (firstGround && secondGround)
                            continue;

                        if (!firstGround && !secondGround)
                        {
                            m_devices[i]->setRotation(SVR_270);
                            alignTerminalEscape(m_devices[i], 0, seed);
                            netAnchors[m_devices[i]->getNode(0)] =
                                getTerminalEscape(m_devices[i], 0);
                            netAnchors[m_devices[i]->getNode(1)] =
                                getTerminalEscape(m_devices[i], 1);
                        }
                        else
                        {
                            const unsigned int nonGround = firstGround ? 1 : 0;
                            alignTerminalEscape(m_devices[i], nonGround, seed);
                            netAnchors[m_devices[i]->getNode(nonGround)] = seed;
                        }

                        placed[i] = true;
                        break;
                    }
                }
            }

            // Attach pull-ups, collector loads, emitter resistors, bypass
            // capacitors, etc. to already anchored signal nets.
            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i] || m_devices[i]->getNodesCount() != 2)
                    continue;

                for (size_t nodeIdx=0; nodeIdx<2; nodeIdx++)
                {
                    const svNode& anchorNode = m_devices[i]->getNode(nodeIdx);
                    const svNode& otherNode = m_devices[i]->getNode(1-nodeIdx);
                    std::map<svNode, wxPoint>::const_iterator anchorIt =
                        netAnchors.find(anchorNode);
                    if (anchorIt == netAnchors.end())
                        continue;

                    if (otherNode == svGroundNode || isPowerNodeName(otherNode))
                    {
                        unsigned int& branch = otherNode == svGroundNode
                            ? groundBranches[anchorNode]
                            : railBranches[anchorNode];
                        bool foundClearLane = false;
                        for (unsigned int attempt=0; attempt<64; attempt++)
                        {
                            const wxPoint desired =
                                anchorIt->second +
                                wxPoint(orderlyBranchLaneOffset(branch++), 0);
                            alignTerminalEscape(m_devices[i], nodeIdx, desired);
                            if (!placementConflicts(i, m_devices, placed))
                            {
                                foundClearLane = true;
                                break;
                            }
                        }
                        if (foundClearLane)
                            placed[i] = true;
                        break;
                    }
                }
            }

            // Grow ordinary two-terminal signal paths left-to-right. Choose
            // the rotation from the anchored terminal so the unanchored end
            // always advances three grid steps to the right. Additional
            // fan-out branches get orderly rows instead of stacking symbols.
            bool progress = true;
            while (progress)
            {
                progress = false;
                for (size_t i=0; i<m_devices.size(); i++)
                {
                    if (placed[i] || m_devices[i]->getNodesCount() != 2)
                        continue;
                    if (m_devices[i]->getNode(0) == svGroundNode ||
                        m_devices[i]->getNode(1) == svGroundNode)
                        continue;
                    if (isPowerNodeName(m_devices[i]->getNode(0)) ||
                        isPowerNodeName(m_devices[i]->getNode(1)))
                        continue;

                    const std::map<svNode, wxPoint>::const_iterator firstAnchor =
                        netAnchors.find(m_devices[i]->getNode(0));
                    const std::map<svNode, wxPoint>::const_iterator secondAnchor =
                        netAnchors.find(m_devices[i]->getNode(1));
                    const bool firstKnown = firstAnchor != netAnchors.end();
                    const bool secondKnown = secondAnchor != netAnchors.end();
                    if (!firstKnown && !secondKnown)
                        continue;

                    unsigned int anchorIdx = 0;
                    if (!firstKnown)
                        anchorIdx = 1;
                    else if (secondKnown && secondAnchor->second.x < firstAnchor->second.x)
                        anchorIdx = 1;

                    const unsigned int otherIdx = 1 - anchorIdx;
                    const svNode& anchorNode = m_devices[i]->getNode(anchorIdx);
                    const svNode& otherNode = m_devices[i]->getNode(otherIdx);
                    const bool otherAlreadyKnown =
                        netAnchors.find(otherNode) != netAnchors.end();

                    // External signal names carry useful flow information.
                    // Keep VIN/input nets toward the left edge and VOUT/output
                    // nets toward the right, even when a passive happens to be
                    // listed in reverse order in the SPICE deck.
                    bool growRight = true;
                    if (isOutputLikeNode(anchorNode) && !isOutputLikeNode(otherNode))
                        growRight = false;
                    else if (isInputLikeNode(otherNode) && !isInputLikeNode(anchorNode))
                        growRight = false;
                    else if (isInputLikeNode(anchorNode) || isOutputLikeNode(otherNode))
                        growRight = true;

                    if (growRight)
                        m_devices[i]->setRotation(anchorIdx == 0 ? SVR_270 : SVR_90);
                    else
                        m_devices[i]->setRotation(anchorIdx == 0 ? SVR_90 : SVR_270);
                    wxPoint desired = netAnchors.find(anchorNode)->second;

                    if (!otherAlreadyKnown)
                    {
                        unsigned int& branch = signalBranches[anchorNode];
                        bool clearLane = false;
                        while (!clearLane)
                        {
                            desired = netAnchors.find(anchorNode)->second;
                            desired.y += orderlyBranchLaneOffset(branch++);
                            alignTerminalEscape(m_devices[i], anchorIdx, desired);

                            const wxPoint candidateOther =
                                getTerminalEscape(m_devices[i], otherIdx);
                            clearLane = true;
                            for (std::map<svNode, wxPoint>::const_iterator
                                 it=netAnchors.begin(); it!=netAnchors.end(); ++it)
                            {
                                if (it->first != m_devices[i]->getNode(otherIdx) &&
                                    it->second == candidateOther)
                                {
                                    clearLane = false;
                                    break;
                                }
                            }
                            if (clearLane && placementConflicts(i, m_devices, placed))
                                clearLane = false;
                        }
                    }
                    else
                    {
                        // This is a back-edge/feedback part: both nets already
                        // exist, so dropping the component directly beside one
                        // anchor stacks it on the forward path. Reserve an
                        // outer row above the occupied core and route both nets
                        // to it later.
                        svPlacementRect occupied;
                        getOccupiedPlacementBounds(m_devices, placed, &occupied);
                        unsigned int& branch = signalBranches[anchorNode];
                        bool clearLane = false;
                        for (unsigned int attempt=0; attempt<64; attempt++)
                        {
                            const int clearance =
                                2 + orderlyBranchLaneOffset(branch++);
                            desired = netAnchors.find(anchorNode)->second;
                            desired.y = occupied.top - clearance;
                            alignTerminalEscape(m_devices[i], anchorIdx, desired);
                            if (!placementConflicts(i, m_devices, placed))
                            {
                                clearLane = true;
                                break;
                            }
                        }
                        if (!clearLane)
                            continue;
                    }

                    placed[i] = true;

                    if (!otherAlreadyKnown)
                        netAnchors[otherNode] =
                            getTerminalEscape(m_devices[i], otherIdx);

                    progress = true;
                }
            }

            // Signal growth can reveal a node only after the first shunt pass.
            // Give newly discovered nets the same orderly ground/rail branch
            // treatment before falling back to the catch-all row. This is
            // especially important for clamp/reference networks where a diode
            // reaches VREF from a signal node discovered later in the chain.
            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i] || m_devices[i]->getNodesCount() != 2)
                    continue;

                for (size_t nodeIdx=0; nodeIdx<2; nodeIdx++)
                {
                    const svNode& anchorNode = m_devices[i]->getNode(nodeIdx);
                    const svNode& otherNode = m_devices[i]->getNode(1-nodeIdx);
                    std::map<svNode, wxPoint>::const_iterator anchorIt =
                        netAnchors.find(anchorNode);
                    if (anchorIt == netAnchors.end())
                        continue;
                    if (otherNode != svGroundNode && !isPowerNodeName(otherNode))
                        continue;

                    unsigned int& branch = otherNode == svGroundNode
                        ? groundBranches[anchorNode]
                        : railBranches[anchorNode];
                    bool foundClearLane = false;
                    for (unsigned int attempt=0; attempt<64; attempt++)
                    {
                        const wxPoint desired =
                            anchorIt->second +
                            wxPoint(orderlyBranchLaneOffset(branch++), 0);
                        alignTerminalEscape(m_devices[i], nodeIdx, desired);
                        if (!placementConflicts(i, m_devices, placed))
                        {
                            foundClearLane = true;
                            break;
                        }
                    }
                    if (foundClearLane)
                        placed[i] = true;
                    break;
                }
            }

            // Place explicit .SUBCKT ports at the edge of the topology instead
            // of dropping them into the fallback row. Input-like ports face
            // inward from the left, output-like ports from the right; generic
            // port names use their anchored position relative to the circuit.
            int minAnchorX = INT_MAX;
            int maxAnchorX = INT_MIN;
            for (std::map<svNode, wxPoint>::const_iterator it=netAnchors.begin();
                 it!=netAnchors.end(); ++it)
            {
                minAnchorX = std::min(minAnchorX, it->second.x);
                maxAnchorX = std::max(maxAnchorX, it->second.x);
            }
            const int centerAnchorX =
                minAnchorX <= maxAnchorX ? (minAnchorX + maxAnchorX) / 2 : 0;
            svPlacementRect pinCoreBounds;
            const bool havePinCoreBounds =
                getOccupiedPlacementBounds(m_devices, placed, &pinCoreBounds);

            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i] || m_devices[i]->getSPICEid() != 0 ||
                    m_devices[i]->getNodesCount() != 1)
                    continue;

                const svNode& node = m_devices[i]->getNode(0);
                std::map<svNode, wxPoint>::const_iterator anchorIt =
                    netAnchors.find(node);
                if (anchorIt == netAnchors.end())
                    continue;

                bool placeRight = isOutputLikeNode(node);
                if (!placeRight && !isInputLikeNode(node))
                    placeRight = anchorIt->second.x > centerAnchorX;

                // SVR_270 puts the port body to the left and its wire outward
                // direction to the right; SVR_90 mirrors that on the right.
                // Put the visible pin beyond the occupied core rather than
                // directly on top of the internal net anchor.
                m_devices[i]->setRotation(placeRight ? SVR_90 : SVR_270);
                bool foundClearLane = false;
                for (unsigned int attempt=0; attempt<64; attempt++)
                {
                    wxPoint desired = anchorIt->second;
                    const int edgeClearance = 2;
                    if (havePinCoreBounds)
                    {
                        desired.x = placeRight
                            ? std::max(anchorIt->second.x + edgeClearance,
                                       pinCoreBounds.right + edgeClearance)
                            : std::min(anchorIt->second.x - edgeClearance,
                                       pinCoreBounds.left - edgeClearance);
                    }
                    else
                    {
                        desired.x += placeRight ? edgeClearance : -edgeClearance;
                    }
                    if (attempt > 0)
                        desired.y += orderlyBranchLaneOffset(attempt);
                    alignTerminalEscape(m_devices[i], 0, desired);
                    if (!placementConflicts(i, m_devices, placed))
                    {
                        foundClearLane = true;
                        break;
                    }
                }
                if (foundClearLane)
                    placed[i] = true;
            }

            // Now that input/output nets have anchors, attach their grounded
            // sources and loads directly below those nodes.
            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i] || m_devices[i]->getNodesCount() != 2)
                    continue;

                int signalIdx = wxNOT_FOUND;
                if (m_devices[i]->getNode(0) == svGroundNode)
                    signalIdx = 1;
                else if (m_devices[i]->getNode(1) == svGroundNode)
                    signalIdx = 0;

                if (signalIdx == wxNOT_FOUND)
                    continue;

                const svNode& signalNode = m_devices[i]->getNode(signalIdx);
                std::map<svNode, wxPoint>::const_iterator anchorIt =
                    netAnchors.find(signalNode);
                if (anchorIt == netAnchors.end() || isPowerNodeName(signalNode))
                    continue;

                unsigned int& branch = groundBranches[signalNode];
                bool foundClearLane = false;
                for (unsigned int attempt=0; attempt<64; attempt++)
                {
                    const wxPoint desired =
                        anchorIt->second +
                        wxPoint(orderlyBranchLaneOffset(branch++), 0);
                    alignTerminalEscape(m_devices[i], signalIdx, desired);
                    if (!placementConflicts(i, m_devices, placed))
                    {
                        foundClearLane = true;
                        break;
                    }
                }
                if (foundClearLane)
                    placed[i] = true;
            }

            // Derive a compact location for grounded power sources from the
            // rail-connected parts already on the page.
            std::map<svNode, wxPoint> powerPlacement;
            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (!placed[i])
                    continue;

                for (size_t nodeIdx=0; nodeIdx<m_devices[i]->getNodesCount(); nodeIdx++)
                {
                    const svNode& node = m_devices[i]->getNode(nodeIdx);
                    if (!isPowerNodeName(node))
                        continue;

                    const wxPoint escape =
                        getTerminalEscape(m_devices[i], nodeIdx);
                    std::map<svNode, wxPoint>::iterator it =
                        powerPlacement.find(node);
                    if (it == powerPlacement.end())
                        powerPlacement[node] = escape;
                    else
                    {
                        it->second.x = std::min(it->second.x, escape.x);
                        it->second.y = std::min(it->second.y, escape.y);
                    }
                }
            }

            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i] || m_devices[i]->getNodesCount() != 2)
                    continue;

                int powerIdx = wxNOT_FOUND;
                if (isPowerNodeName(m_devices[i]->getNode(0)) &&
                    m_devices[i]->getNode(1) == svGroundNode)
                    powerIdx = 0;
                else if (isPowerNodeName(m_devices[i]->getNode(1)) &&
                         m_devices[i]->getNode(0) == svGroundNode)
                    powerIdx = 1;

                if (powerIdx == wxNOT_FOUND)
                    continue;

                const svNode& powerNode = m_devices[i]->getNode(powerIdx);
                wxPoint desired(4, 1);
                std::map<svNode, wxPoint>::const_iterator it =
                    powerPlacement.find(powerNode);
                if (it != powerPlacement.end())
                    desired = wxPoint(it->second.x - 2, it->second.y);

                bool foundClearLane = false;
                for (unsigned int attempt=0; attempt<64; attempt++)
                {
                    wxPoint candidate = desired - wxPoint((int)attempt * 2, 0);
                    alignTerminalEscape(m_devices[i], powerIdx, candidate);
                    if (!placementConflicts(i, m_devices, placed))
                    {
                        desired = candidate;
                        foundClearLane = true;
                        break;
                    }
                }
                if (foundClearLane)
                {
                    placed[i] = true;
                    netAnchors[powerNode] = desired;
                }
            }

            // Unusual unsupported topology falls back to a compact row below
            // the main schematic instead of overlapping the primary drawing.
            int fallbackX = 3;
            const int fallbackY = 12 + (int)transistorCount * 2;
            for (size_t i=0; i<m_devices.size(); i++)
            {
                if (placed[i])
                    continue;

                const int left = m_devices[i]->getLeftmostGridNodePosition();
                const int right = m_devices[i]->getRightmostGridNodePosition();
                int candidateX = fallbackX;
                while (true)
                {
                    m_devices[i]->setGridPosition(
                        wxPoint(candidateX-left, fallbackY));
                    if (!placementConflicts(i, m_devices, placed))
                        break;
                    candidateX += 2;
                }

                placed[i] = true;
                fallbackX = candidateX + std::max(3, right-left+3);
            }
        }
        break;

    case SVPA_KAMADA_KAWAI:
        {
            svUGraph graph = buildGraph();
            //svUGraph::PositionMap map;
            
            typedef struct 
            {
                double x, y;
            } point2D;

            //typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
//            boost::property_map<svUGraph, point2D> map;// = g�et(boost::vertex_index, graph);
        //    boost::property_map<svUGraph, vertex_id_t>::type vertex_id_map; 
//            circle_graph_layout(graph, map, 20.0);
        }
        break;

    case SVPA_HEURISTIC_1:
        {
            // start placing the first device at the center of our virtual grid
            // (we place it using its first node as reference)
            for (unsigned int i=0; i<m_devices.size(); i++)
                m_devices[i]->setGridPosition(wxPoint(0,0));

            // now place other devices connected to the same nodes of the first device:
            for (size_t j=0; j<m_devices[0]->getNodesCount(); j++)
            {
                // TODO: verify the position is free
                // TODO: cycle on the nodes, not on the devices! first place close together
                //       all devices attached to the same node

                if (m_devices[0]->getNode(j) != svGroundNode)        // node 0 == GND
                {
                    unsigned int temp;
                    for (size_t i=1; i<m_devices.size(); i++)
                        if (m_devices[i]->isConnectedTo(m_devices[0]->getNode(j), &temp))
                        {
                            // place it to the right of the previous device so that it can be easily
                            // connected...
                            wxPoint pos = m_devices[0]->getGridPosition() + wxPoint(1,0);
                            pos.x -= m_devices[i]->getLeftmostGridNodePosition();
                            pos.y += m_devices[i]->getRelativeGridNodePosition(temp).y;
                            m_devices[i]->setGridPosition(pos);
                            break;
                        }
                
                    break;
                }
            }

            // TODO: finish placement of other devices
        }
        break;
    }

    // define the translation values to use to make all grid points positive:
    wxPoint offset;
    for (size_t i=0; i<m_devices.size(); i++)
    {
        wxPoint pt = m_devices[i]->getGridPosition();
        offset.x = std::min(offset.x, pt.x + m_devices[i]->getLeftmostGridNodePosition());
        offset.y = std::min(offset.y, pt.y + m_devices[i]->getTopmostGridNodePosition());
    }
    // Keep room for terminal stubs, shared-net doglegs, ground symbols and
    // horizontal annotations around the automatically placed drawing.
    offset = wxPoint(5,5) + offset*(-1);

    for (size_t i=0; i<m_devices.size(); i++)
        m_devices[i]->setGridPosition(m_devices[i]->getGridPosition() + offset);

    updateBoundingBox();
    return m_bb;
}

void svCircuit::updateBoundingBox()
{
    m_bb.x = m_bb.y = INT_MAX-1;
    m_bb.width = m_bb.height = INT_MIN+1;
    for (size_t i=0; i<m_devices.size(); i++)
    {
        m_bb.x = std::min(m_devices[i]->getGridPosition().x +
                          m_devices[i]->getLeftmostGridNodePosition(), m_bb.x);
        m_bb.y = std::min(m_devices[i]->getGridPosition().y +
                          m_devices[i]->getTopmostGridNodePosition(), m_bb.y);

        m_bb.width = std::max(m_devices[i]->getGridPosition().x +
                              m_devices[i]->getRightmostGridNodePosition(), m_bb.width);
        m_bb.height = std::max(m_devices[i]->getGridPosition().y +
                               m_devices[i]->getBottommostGridNodePosition(), m_bb.height);
    }

    m_bb.width -= m_bb.x;
    m_bb.height -= m_bb.y;
}

void svCircuit::initGraphics(wxGraphicsContext*gc, unsigned int gridSize)
{
    s_pathGround = gc->CreatePath();

    double w = gridSize/3.0, d = gridSize/10.0;
    drawLine(s_pathGround, wxRealPoint(-w,0), wxRealPoint(w,0));
    drawLine(s_pathGround, wxRealPoint(-w*2/4,d), wxRealPoint(w*2/4,d));
    drawLine(s_pathGround, wxRealPoint(-w*1/4,2*d), wxRealPoint(w*1/4,2*d));
}

void svCircuit::draw(wxGraphicsContext* gc, unsigned int gridSize, int selectedDevice) const
{
    // draw all the devices
    wxPen normal(*wxBLACK, 2),
          selected(*wxRED, 2);
    gc->SetFont(*wxSWISS_FONT, *wxBLACK);
    for (size_t i=0; i<m_devices.size(); i++)
    {
        const wxPen& devicePen = selectedDevice == (int)i ? selected : normal;
        m_devices[i]->drawWithDesc(gc, gridSize, devicePen);

#if 0
        // draw the bounding box for each device
        gc->SetTransform(gc->CreateMatrix());   // reset the transformation matrix
        wxRect r = m_devices[i]->getRealBoundingBox(gridSize);
        gc->SetPen(selected);
        gc->SetBrush(*wxTRANSPARENT_BRUSH);
        gc->DrawRectangle(r.x, r.y, r.width, r.height);
#endif

        // decorate the nodes of this device
        for (size_t j=0; j<m_devices[i]->getNodesCount(); j++)
        {
            const wxPoint gridNodePos =
                m_devices[i]->getGridPosition() + m_devices[i]->getRelativeGridNodePosition(j);
            wxRealPoint nodePos = gridNodePos * gridSize;

            // reset the transformation matrix
            wxGraphicsMatrix m = gc->CreateMatrix();

            if (m_devices[i]->getNode(j) == svGroundNode)
            {
                const wxPoint direction = m_devices[i]->getRelativeGridNodeDirection(j);
                const wxPoint groundPos = gridNodePos + direction;

                gc->SetTransform(m);
                gc->SetPen(devicePen);
                drawGridLine(gc, gridNodePos, groundPos, gridSize);

                m.Translate(groundPos.x * gridSize, groundPos.y * gridSize);
                m.Rotate(getGroundRotation(direction));
                gc->SetTransform(m);
                gc->StrokePath(s_pathGround);
            }
            // Internal node names are deliberately not stamped at every
            // terminal. Connectivity should be evident from the wiring and
            // junction dots; external pins still label themselves.
        }
    }

    // Conventional schematics use a single neutral wire colour. Net identity
    // comes from topology, junctions and labels, not rainbow routing.
    wxPen wirePen(*wxBLACK, 2);

    // reset transformation matrix:
    gc->SetTransform(gc->CreateMatrix());
    const svGridPointSet blockedGroundRouting =
        collectGroundRoutingBlocks(m_devices);
    const svRoutingObstacleArray routingObstacles =
        collectDeviceRoutingObstacles(m_devices, gridSize);

    for (std::set<svNode>::const_iterator i=m_nodes.begin(); i != m_nodes.end(); i++)
    {
        if (*i != svGroundNode)
        {
            gc->SetPen(wirePen);

            std::vector<svWireTerminal> terminals;
            for (size_t deviceIdx=0; deviceIdx<m_devices.size(); deviceIdx++)
            {
                for (size_t nodeIdx=0; nodeIdx<m_devices[deviceIdx]->getNodesCount(); nodeIdx++)
                {
                    if (m_devices[deviceIdx]->getNode(nodeIdx) != *i)
                        continue;

                    svWireTerminal terminal;
                    terminal.node = m_devices[deviceIdx]->getGridPosition() +
                                    m_devices[deviceIdx]->getRelativeGridNodePosition(nodeIdx);
                    terminal.direction = m_devices[deviceIdx]->getRelativeGridNodeDirection(nodeIdx);
                    terminal.escape = terminal.node + terminal.direction;
                    terminal.externalPin = m_devices[deviceIdx]->getSPICEid() == 0;
                    terminals.push_back(terminal);
                }
            }

            // Named supply rails are clearer as conventional global power
            // markers than as a long physical wire spanning the drawing.
            if (isPowerNodeName(*i))
            {
                for (size_t j=0; j<terminals.size(); j++)
                    drawPowerNetMarker(gc, terminals[j], *i, gridSize);
                continue;
            }

            if (terminals.size() == 2)
            {
                // Every terminal owns exactly one straight grid segment before
                // a two-terminal route can bend.
                for (size_t j=0; j<terminals.size(); j++)
                    drawGridLine(gc, terminals[j].node, terminals[j].escape, gridSize);
                drawOrthogonalConnection(
                    gc, terminals[0], terminals[1], blockedGroundRouting,
                    routingObstacles, gridSize);
            }
            else if (terminals.size() > 2)
            {
                // Multi-terminal nets share one scored Manhattan trunk. The
                // tree deduplicates overlapping grid edges and junction dots
                // are derived from actual branch degree, not a synthetic hub.
                drawSharedManhattanTree(
                    gc, terminals, blockedGroundRouting, routingObstacles,
                    gridSize);
            }
            // A one-terminal net has no electrical connection to draw. The
            // component pin itself is sufficient; drawing node->escape here
            // would create a fake floating wire stub.

            drawUsefulSignalLabel(gc, terminals, *i, gridSize);
        }
    }
}

std::vector<wxPoint> svCircuit::getDeviceNodesConnectedTo(const svNode& node) const
{
    std::vector<wxPoint> ret;
    for (size_t i = 0; i < m_devices.size(); i++)
    {
        wxPoint pt = m_devices[i]->getRelativeGridNodePosition(node);
        if (pt != svInvalidPoint)
            ret.push_back(m_devices[i]->getGridPosition() + pt);
    }
    return ret;
}

void svCircuit::assign(const svCircuit& tocopy)
{
    release();
    m_name = tocopy.m_name;
    m_nodes = tocopy.m_nodes;
    m_bb = tocopy.m_bb;
    for (size_t i = 0; i < tocopy.m_devices.size(); i++)
        m_devices.push_back(tocopy.m_devices[i]->clone());
}

void svCircuit::release()
{
    for (size_t i = 0; i < m_devices.size(); i++)
        delete m_devices[i];
    m_devices.clear();
    m_name.clear();
    m_nodes.clear();
    m_bb = wxRect(0, 0, 0, 0);
}

int svCircuit::hitTest(const wxPoint& gridPt, unsigned int gridSize, unsigned int tolerance) const
        {
    for (size_t i = 0; i < m_devices.size(); i++)
    {
        wxRect r = m_devices[i]->getRealBoundingBox(gridSize);
        r.Inflate(tolerance, tolerance);
        if (r.x < gridPt.x && r.y < gridPt.y && r.x + r.width >= gridPt.x && r.y + r.height >= gridPt.y)
            return i;
    }
    return wxNOT_FOUND;
}
