//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#if defined(WITH_OSG) && defined(WITH_OSGEARTH)

#include "Satellite.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"
#include "Dijkstra.h"
#include "GroundStation.h"


#include <sstream>
#include <iomanip>

#include "omnetpp/osgutil.h"

#include <osg/Node>
#include <osg/Texture>
#include <osg/LineWidth>
#include <osg/PolygonMode>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/Depth>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Geometry>
#include <osgEarthFeatures/Feature>

using namespace omnetpp;

Define_Module(Satellite)

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

Graph graph = {
    {{1, 1},{10, 1},{11, 1},{55, 1},{66, 1},{67, 1}},// 节点0连接的节点及权重
    {{0, 1},{2, 1},{12, 1},{56, 1},{66, 1},{67, 1}},// 节点1连接的节点及权重
    {{1, 1},{3, 1},{13, 1},{57, 1},{66, 1},{67, 1}},// 节点2连接的节点及权重
    {{2, 1},{4, 1},{14, 1},{58, 1},{66, 1},{67, 1}},// 节点3连接的节点及权重
    {{3, 1},{5, 1},{15, 1},{59, 1},{66, 1},{67, 1}},// 节点4连接的节点及权重
    {{4, 1},{6, 1},{16, 1},{60, 1},{66, 1},{67, 1}},// 节点5连接的节点及权重
    {{5, 1},{7, 1},{17, 1},{61, 1},{66, 1},{67, 1}},// 节点6连接的节点及权重
    {{6, 1},{8, 1},{18, 1},{62, 1},{66, 1},{67, 1}},// 节点7连接的节点及权重
    {{7, 1},{9, 1},{19, 1},{63, 1},{66, 1},{67, 1}},// 节点8连接的节点及权重
    {{8, 1},{10, 1},{20, 1},{64, 1},{66, 1},{67, 1}},// 节点9连接的节点及权重
    {{0, 1},{9, 1},{21, 1},{65, 1},{66, 1},{67, 1}},// 节点10连接的节点及权重
    {{0, 1},{12, 1},{21, 1},{22, 1},{66, 1},{67, 1}},// 节点11连接的节点及权重
    {{1, 1},{11, 1},{13, 1},{23, 1},{66, 1},{67, 1}},// 节点12连接的节点及权重
    {{2, 1},{12, 1},{14, 1},{24, 1},{66, 1},{67, 1}},// 节点13连接的节点及权重
    {{3, 1},{13, 1},{15, 1},{25, 1},{66, 1},{67, 1}},// 节点14连接的节点及权重
    {{4, 1},{14, 1},{16, 1},{26, 1},{66, 1},{67, 1}},// 节点15连接的节点及权重
    {{5, 1},{15, 1},{17, 1},{27, 1},{66, 1},{67, 1}},// 节点16连接的节点及权重
    {{6, 1},{16, 1},{18, 1},{28, 1},{66, 1},{67, 1}},// 节点17连接的节点及权重
    {{7, 1},{17, 1},{19, 1},{29, 1},{66, 1},{67, 1}},// 节点18连接的节点及权重
    {{8, 1},{18, 1},{20, 1},{30, 1},{66, 1},{67, 1}},// 节点19连接的节点及权重
    {{9, 1},{19, 1},{21, 1},{31, 1},{66, 1},{67, 1}},// 节点20连接的节点及权重
    {{10, 1},{11, 1},{20, 1},{32, 1},{66, 1},{67, 1}},// 节点21连接的节点及权重
    {{11, 1},{23, 1},{32, 1},{33, 1},{66, 1},{67, 1}},// 节点22连接的节点及权重
    {{12, 1},{22, 1},{24, 1},{34, 1},{66, 1},{67, 1}},// 节点23连接的节点及权重
    {{13, 1},{23, 1},{25, 1},{35, 1},{66, 1},{67, 1}},// 节点24连接的节点及权重
    {{14, 1},{24, 1},{26, 1},{36, 1},{66, 1},{67, 1}},// 节点25连接的节点及权重
    {{15, 1},{25, 1},{27, 1},{37, 1},{66, 1},{67, 1}},// 节点26连接的节点及权重
    {{16, 1},{26, 1},{28, 1},{38, 1},{66, 1},{67, 1}},// 节点27连接的节点及权重
    {{17, 1},{27, 1},{29, 1},{39, 1},{66, 1},{67, 1}},// 节点28连接的节点及权重
    {{18, 1},{28, 1},{30, 1},{40, 1},{66, 1},{67, 1}},// 节点29连接的节点及权重
    {{19, 1},{29, 1},{31, 1},{41, 1},{66, 1},{67, 1}},// 节点30连接的节点及权重
    {{20, 1},{30, 1},{32, 1},{42, 1},{66, 1},{67, 1}},// 节点31连接的节点及权重
    {{21, 1},{22, 1},{31, 1},{43, 1},{66, 1},{67, 1}},// 节点32连接的节点及权重
    {{22, 1},{34, 1},{43, 1},{44, 1},{66, 1},{67, 1}},// 节点33连接的节点及权重
    {{23, 1},{33, 1},{35, 1},{45, 1},{66, 1},{67, 1}},// 节点34连接的节点及权重
    {{24, 1},{34, 1},{36, 1},{46, 1},{66, 1},{67, 1}},// 节点35连接的节点及权重
    {{25, 1},{35, 1},{37, 1},{47, 1},{66, 1},{67, 1}},// 节点36连接的节点及权重
    {{26, 1},{36, 1},{38, 1},{48, 1},{66, 1},{67, 1}},// 节点37连接的节点及权重
    {{27, 1},{37, 1},{39, 1},{49, 1},{66, 1},{67, 1}},// 节点38连接的节点及权重
    {{28, 1},{38, 1},{40, 1},{50, 1},{66, 1},{67, 1}},// 节点39连接的节点及权重
    {{29, 1},{39, 1},{41, 1},{51, 1},{66, 1},{67, 1}},// 节点40连接的节点及权重
    {{30, 1},{40, 1},{42, 1},{52, 1},{66, 1},{67, 1}},// 节点41连接的节点及权重
    {{31, 1},{41, 1},{43, 1},{53, 1},{66, 1},{67, 1}},// 节点42连接的节点及权重
    {{32, 1},{33, 1},{42, 1},{54, 1},{66, 1},{67, 1}},// 节点43连接的节点及权重
    {{33, 1},{45, 1},{54, 1},{55, 1},{66, 1},{67, 1}},// 节点44连接的节点及权重
    {{34, 1},{44, 1},{46, 1},{56, 1},{66, 1},{67, 1}},// 节点45连接的节点及权重
    {{35, 1},{45, 1},{47, 1},{57, 1},{66, 1},{67, 1}},// 节点46连接的节点及权重
    {{36, 1},{46, 1},{48, 1},{58, 1},{66, 1},{67, 1}},// 节点47连接的节点及权重
    {{37, 1},{47, 1},{49, 1},{59, 1},{66, 1},{67, 1}},// 节点48连接的节点及权重
    {{38, 1},{48, 1},{50, 1},{60, 1},{66, 1},{67, 1}},// 节点49连接的节点及权重
    {{39, 1},{49, 1},{51, 1},{61, 1},{66, 1},{67, 1}},// 节点50连接的节点及权重
    {{40, 1},{50, 1},{52, 1},{62, 1},{66, 1},{67, 1}},// 节点51连接的节点及权重
    {{41, 1},{51, 1},{53, 1},{63, 1},{66, 1},{67, 1}},// 节点52连接的节点及权重
    {{42, 1},{52, 1},{54, 1},{64, 1},{66, 1},{67, 1}},// 节点53连接的节点及权重
    {{43, 1},{44, 1},{53, 1},{65, 1},{66, 1},{67, 1}},// 节点54连接的节点及权重
    {{0, 1},{44, 1},{56, 1},{65, 1},{66, 1},{67, 1}},// 节点55连接的节点及权重
    {{1, 1},{45, 1},{55, 1},{57, 1},{66, 1},{67, 1}},// 节点56连接的节点及权重
    {{2, 1},{46, 1},{56, 1},{58, 1},{66, 1},{67, 1}},// 节点57连接的节点及权重
    {{3, 1},{47, 1},{57, 1},{59, 1},{66, 1},{67, 1}},// 节点58连接的节点及权重
    {{4, 1},{48, 1},{58, 1},{60, 1},{66, 1},{67, 1}},// 节点59连接的节点及权重
    {{5, 1},{49, 1},{59, 1},{61, 1},{66, 1},{67, 1}},// 节点60连接的节点及权重
    {{6, 1},{50, 1},{60, 1},{62, 1},{66, 1},{67, 1}},// 节点61连接的节点及权重
    {{7, 1},{51, 1},{61, 1},{63, 1},{66, 1},{67, 1}},// 节点62连接的节点及权重
    {{8, 1},{52, 1},{62, 1},{64, 1},{66, 1},{67, 1}},// 节点63连接的节点及权重
    {{9, 1},{53, 1},{63, 1},{65, 1},{66, 1},{67, 1}},// 节点64连接的节点及权重
    {{10, 1},{54, 1},{55, 1},{64, 1},{66, 1},{67, 1}},// 节点65连接的节点及权重
    {{0, 1},{1, 1},{2, 1},{3, 1},{4, 1},{5, 1},{6, 1},{7, 1},{8, 1},{9, 1},{10, 1},{11, 1},{12, 1},{13, 1},{14, 1},{15, 1},{16, 1},{17, 1},{18, 1},{19, 1},{20, 1},{21, 1},{22, 1},{23, 1},{24, 1},{25, 1},{26, 1},{27, 1},{28, 1},{29, 1},{30, 1},{31, 1},{32, 1},{33, 1},{34, 1},{35, 1},{36, 1},{37, 1},{38, 1},{39, 1},{40, 1},{41, 1},{42, 1},{43, 1},{44, 1},{45, 1},{46, 1},{47, 1},{48, 1},{49, 1},{50, 1},{51, 1},{52, 1},{53, 1},{54, 1},{55, 1},{56, 1},{57, 1},{58, 1},{59, 1},{60, 1},{61, 1},{62, 1},{63, 1},{64, 1},{65,1}},// 节点66连接的节点及权重
    {{0, 1},{1, 1},{2, 1},{3, 1},{4, 1},{5, 1},{6, 1},{7, 1},{8, 1},{9, 1},{10, 1},{11, 1},{12, 1},{13, 1},{14, 1},{15, 1},{16, 1},{17, 1},{18, 1},{19, 1},{20, 1},{21, 1},{22, 1},{23, 1},{24, 1},{25, 1},{26, 1},{27, 1},{28, 1},{29, 1},{30, 1},{31, 1},{32, 1},{33, 1},{34, 1},{35, 1},{36, 1},{37, 1},{38, 1},{39, 1},{40, 1},{41, 1},{42, 1},{43, 1},{44, 1},{45, 1},{46, 1},{47, 1},{48, 1},{49, 1},{50, 1},{51, 1},{52, 1},{53, 1},{54, 1},{55, 1},{56, 1},{57, 1},{58, 1},{59, 1},{60, 1},{61, 1},{62, 1},{63, 1},{64, 1},{65,1}},// 节点67连接的节点及权重
};

Graph position = {
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, 1},{1, 1},{2, 1},},
    {{0, -21.9187},{1, 51.7908},{2, 29.9881},},
    {{0, -21.7777},{1, 43.8878},{2, 40.7},},
};

Path shortestPath;
std::vector<Path> shortestPaths;
Dijkstra dijkstra;
GroundStation groundStation;
std::vector<int> loadCount(66,0);
int maxLoad = 3;
omnetpp::simtime_t lastUpdateTime;
RHPP rhpp;

void Satellite::initialize(int stage)
{
    lastUpdateTime = omnetpp::simTime();
    switch (stage) {
    case 0: {
        modelURL = par("modelURL").stringValue();
        modelScale = par("modelScale");
        labelColor = par("labelColor").stringValue();
        altitude = par("altitude");

        phase = startingPhase = par("startingPhase").doubleValue() * M_PI / 180.0;

        std::string normalString = par("orbitNormal");
        if (normalString.empty()) {
            // it is not a correct spherical distribution, nor deterministic, but will do here
             normal.set(dblrand() * 2 - 1, dblrand() * 2 - 1, dblrand() * 2 - 1);
        }
        else {
            std::stringstream ss(normalString);

            double x, y, z;
            ss >> x;
            ss.ignore();
            ss >> y;
            ss.ignore();
            ss >> z;

            if (!ss)
                throw cRuntimeError("Couldn't parse orbit normal vector \"%s\", the correct format is for example \"2.5,3,0\", or leave it empty for random", normalString.c_str());

            normal.set(x, y, z);
        }

        normal.normalize();

        auto c1 = normal ^ osg::Vec3d(0, 1, 0);
        auto c2 = normal ^ osg::Vec3d(1, 0, 0);
        osg::Vec3d &cross = c1.length2() < 0.1 ? c2 : c1;

        cross.normalize();
        orbitX = cross;
        orbitY = normal ^ cross;

        getOsgCanvas()->setScene(osgDB::readNodeFile(modelURL));

        break;
    }
    case 1:
        ChannelController::getInstance()->addSatellite(this);

        auto scene = OsgEarthScene::getInstance()->getScene(); // scene is initialized in stage 0 so we have to do our init in stage 1
        mapNode = osgEarth::MapNode::findMapNode(scene);

        // build up the node representing this module
        // a GeoTransform allows positioning a model using world coordinates
        geoTransform = new osgEarth::GeoTransform();

        auto modelNode = osgDB::readNodeFile(modelURL);

        modelNode->getOrCreateStateSet()->setAttributeAndModes(
          new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
        modelNode->getOrCreateStateSet()->setMode(
                  GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

        // scale and rotate the model if necessary
        auto pat = new osg::PositionAttitudeTransform();
        pat->setScale(osg::Vec3d(modelScale, modelScale, modelScale));

        auto objectNode = new cObjectOsgNode(this);
        pat->addChild(objectNode);
        objectNode->addChild(modelNode);
        geoTransform->addChild(pat);

        // set the name label if the color is specified
        if (!labelColor.empty()) {
            Style labelStyle;
            labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_TOP;
            labelStyle.getOrCreate<TextSymbol>()->declutter() = true;
            labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = osg::Vec2s(0,50);
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(labelColor);
            geoTransform->addChild(new LabelNode(getFullName(), labelStyle));
        }

        // add the locator node to the scene
        scene->asGroup()->addChild(geoTransform);

        // making the orbit circle
        std::string orbitColor = par("orbitColor");
        if (!orbitColor.empty()) {
            osg::ref_ptr<osg::Geometry> orbitGeom = new osg::Geometry;
            osg::ref_ptr<osg::DrawArrays> drawArrayLines = new  osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
            osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

            orbitGeom->addPrimitiveSet(drawArrayLines);
            auto stateSet = orbitGeom->getOrCreateStateSet();
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
            stateSet->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
            stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            stateSet->setAttributeAndModes(new osg::LineWidth(1.5), osg::StateAttribute::ON);
            stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            auto depth = new osg::Depth;
            depth->setWriteMask(false);
            stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

            for (int i = 0; i <= 100; ++i)
                vertexData->push_back(getPositionAtPhase(i / 100.0 * M_PI*2));

            orbitGeom->setVertexArray(vertexData);
            drawArrayLines->setFirst(0);
            drawArrayLines->setCount(vertexData->size());

            osg::ref_ptr<osg::Vec4Array> colorData = new osg::Vec4Array;
            colorData->push_back(osgEarth::Color(orbitColor));
            orbitGeom->setColorArray(colorData, osg::Array::BIND_OVERALL);

            osg::ref_ptr<osg::Geode> orbitGeode = new osg::Geode;
            orbitGeode->addDrawable(orbitGeom.get());
            scene->asGroup()->addChild(orbitGeode);
        }

        std::string coneColor = par("coneColor");
        if (!coneColor.empty()) {
            double orbitRadius = earthRadius + altitude; // in kilometers
            // the angle between the center of the earth and the horizon as seen from the satellite, in radians
            double alpha = std::asin(earthRadius / orbitRadius);
            // the distance of the horizon from the satellite, in meters
            double horizonDistance = std::sqrt(orbitRadius * orbitRadius - earthRadius * earthRadius) * 1000;
            double coneHeight = std::sin(alpha)*horizonDistance;
            double coneRadius = std::cos(alpha)*horizonDistance;
            // the offset is to position the tip to the satellite
            osg::Cone *cone = new osg::Cone(osg::Vec3(0, 0, -coneRadius*0.75), coneHeight, coneRadius);

            osg::ref_ptr<osg::Geode> coneGeode = new osg::Geode;
            auto coneDrawable = new osg::ShapeDrawable(cone);
            coneDrawable->setColor(osgEarth::Color(coneColor));

            coneGeode->addDrawable(coneDrawable);
            coneGeode->getOrCreateStateSet()->setAttribute(new osg::PolygonMode(
                osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
            coneGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            auto depth = new osg::Depth;
            depth->setWriteMask(false);
            coneGeode->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
            geoTransform->addChild(coneGeode);
        }
    }
    double x, y, z;
    dijkstra.convertGeodeticToECEF(-62.21639, -58.96444, 0, x, y, z);
    position[66][0].weight = x;
    position[66][1].weight = y;
    position[66][2].weight = z;

    cMessage *msg = new cMessage("Ms");
    EV << "Sending initial message\n" << msg->getName();

//    ExtractedData data = extractData(getFullName());
//    int this_sat = data.number;
//
//    // 使用find函数在vector中查找目标值
//    auto path = shortestPath.nodes;
//    auto it = find(path.begin(), path.end(), this_sat);
//    if (it != path.end()) {
//        int index = distance(path.begin(), it);
//        int nextGate = getGate(shortestPath.nodes[index+1]);
//        EV << nextGate;
//        send(msg, "gate$o", nextGate);
//    } else {
    send(msg, "gate$o", uniform(0, 4));
//    }
}

void Satellite::handleMessage(cMessage *msg)
{
//    throw cRuntimeError("This module does not process messages");

    EV << msg->getArrivalModule() << ": Received message from "<< msg->getSenderModule() << ", sending it out again\n";
    ExtractedData data = extractData(getFullName());
    int this_sat = data.number;

    // 使用find函数在vector中查找目标值
    auto path = shortestPath.nodes;
    auto it = find(path.begin(), path.end(), this_sat);
    if (it != path.end()) {
        int index = distance(path.begin(), it);
        int nextGate = getGate(shortestPath.nodes[index+1]);
        send(msg, "gate$o", nextGate);
    } else {
        send(msg, "gate$o", uniform(0, 4));
    }
//    generateGraph();
}

void Satellite::updatePosition()
{
    //double t = simTime().inUnit(SIMTIME_MS) / 1000.0;  ?????
    simtime_t t = simTime();
    if (t != lastPositionUpdateTime) {
        phase = startingPhase + t.dbl() * getOmega();  //FIXME getOmega(); ???
        pos = getPositionAtPhase(phase);

        osg::Vec3d v;
        mapNode->getMapSRS()->transformFromWorld(pos, v);
        geoTransform->setPosition(osgEarth::GeoPoint(mapNode->getMapSRS(), v));

        lastPositionUpdateTime = t;

        // 更新位置信息
        std::string moduleName = getFullName();
        size_t start_pos = moduleName.find('[');
        size_t end_pos = moduleName.find(']');
        int this_sat = 0;
        if (start_pos != std::string::npos && end_pos != std::string::npos) {
            // 截取 "[" 和 "]" 之间的子字符串
            std::string number_str = moduleName.substr(start_pos + 1, end_pos - start_pos - 1);

            // 将子字符串转换为整数
            this_sat = std::stoi(number_str);

//            // 打印结果
//            EV << "Extracted number: " << this_sat << "\n";
        } else {
            EV << "Unable to find '[' or ']'" << "\n";
        }

        position[this_sat][0].weight = pos.x() / 100000;
        position[this_sat][1].weight = pos.y() / 100000;
        position[this_sat][2].weight = pos.z() / 100000;

        if (this_sat == 65){
            EV << "************************************************************\n";
            for (int i = 0; i < graph.size(); i++){
                for (int j = 0; j < graph[i].size(); j++){
                    if (isConnected(graph, i, graph[i][j].to)){
                        double diff_x = (position[i][0].weight - position[graph[i][j].to][0].weight)*(position[i][0].weight - position[graph[i][j].to][0].weight);
                        double diff_y = (position[i][1].weight - position[graph[i][j].to][1].weight)*(position[i][1].weight - position[graph[i][j].to][1].weight);
                        double diff_z = (position[i][2].weight - position[graph[i][j].to][2].weight)*(position[i][2].weight - position[graph[i][j].to][2].weight);
                        graph[i][j].weight = std::sqrt(diff_x + diff_y + diff_z);
                    }
                }
            }

            ChannelController *channelcontroller = ChannelController::getInstance();
            std::vector<std::string> connection = channelcontroller->getConnection();
            for (int i = 0; i < 66; i++){
                for (int j = 0; j < 2; j++){
                    std::string this_connection = "sat[" + std::to_string(i) + "]:" + "stat[" + std::to_string(j) + "]";
                    if (std::find(connection.begin(), connection.end(), this_connection) != connection.end()){
//                        EV << "Found " << this_connection << "\n";
                    } else {
                        graph[66+j][i].weight = -1;
                        graph[i][4+j].weight = -1;
                    }
                }
            }

//            for (int i = 0; i < graph.size(); i++){
//                for (int j = 0; j < graph[i].size(); j++){
//                    if (isConnected(graph, i, graph[i][j].to)){
//                        EV << "sat" << i << ": connect to " << graph[i][j].to << ", weight = " << graph[i][j].weight << "\n";
//                    }
//                }
//                EV << "\n";
//            }
            Path temp_shortestPath = dijkstra.dijkstra(graph, 66, 67);
            shortestPath = rhpp.findPath(graph, temp_shortestPath.nodes[1], temp_shortestPath.nodes[temp_shortestPath.nodes.size()-2]);
            shortestPath.distance = temp_shortestPath.distance;
            EV << "Shortest path from node " << 66 << " to node " << 67 << ": ";
            for (int node : shortestPath.nodes) {
                EV << node << " -> ";
            }
            EV << "\nShortest path distance: " << shortestPath.distance;
            EV << "\nLatency: " << ((shortestPath.distance * 100000) / 300000000) * 1000 << "\n";

//            omnetpp::simtime_t currentTime = omnetpp::simTime();  // 获取当前模拟时间
//            omnetpp::simtime_t timeInterval = currentTime - lastUpdateTime;  // 计算时间间隔
//            EV << "Time interval between updates: " << timeInterval << " seconds.\n";
//
//            lastUpdateTime = currentTime;  // 更新最后更新时间记录
        }
    }
}

osg::Vec3 Satellite::getPositionAtPhase(double alpha) const
{
    return (orbitX * std::cos(alpha) + orbitY * std::sin(alpha)) * (earthRadius + altitude) * 1000;
}

void Satellite::refreshDisplay() const
{
    const_cast<Satellite *>(this)->updatePosition();

    // update the position on the 2D canvas
    getDisplayString().setTagArg("p", 0, 300 + pos.x() / 100000);
    getDisplayString().setTagArg("p", 1, 300 - pos.y() / 100000);
}

int Satellite::getGate(int targetNode){
    // 遍历该模块的所有 gate
    for (int i = 0; i < gateCount(); ++i) {
        cGate* gate = gateByOrdinal(i);
        if (gate->isConnectedOutside()) {  // 检查 gate 是否向外部连接
            cGate* nextGate = gate->getNextGate();
            if (nextGate) {
                cModule* connectedModule = nextGate->getOwnerModule();
                ExtractedData data1 = extractData(gate->getFullName());
                ExtractedData data2 = extractData(connectedModule->getFullName());
                if (data2.prefix == "sat" && data2.number == targetNode){
                    EV << nextGate->getFullName();
                    return 0;
                }
//                EV << "Gate '" << data1.number << "' is connected to " << data2.prefix << data2.number << "\n";
            }
        }
    }
    return 0;
}

bool Satellite::isConnected(const Graph& graph, int node1, int node2) {
    for (const Edge& edge : graph[node1]) {
        if (edge.to == node2) {
            return true; // 存在连接
        }
    }
    return false; // 不存在连接
}

ExtractedData Satellite::extractData(const std::string& str) {
    ExtractedData result;
    size_t start = str.find('[') + 1;  // 找到 '[' 字符，然后偏移到 '[' 之后
    size_t end = str.find(']', start);  // 从 '[' 之后找到 ']'
    if (start != std::string::npos && end != std::string::npos) {
        result.number = std::stoi(str.substr(start, end - start));  // 提取数字并转换为整数
        result.prefix = str.substr(0, start - 1);  // 提取从开始到 '[' 之前的部分作为前缀
    }
    return result;
}

#endif // WITH_OSG
