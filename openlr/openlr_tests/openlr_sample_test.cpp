#include "testing/testing.hpp"

#include "openlr/openlr_sample.hpp"
#include "openlr/openlr_simple_parser.hpp"

#include "indexer/index.hpp"

#include "platform/platform_tests_support/scoped_file.hpp"

#include "3party/pugixml/src/pugixml.hpp"

using namespace openlr;

UNIT_TEST(ParseOpenlr)
{
  auto const openlrData = "<?xml version=\"1.0\"?>"
      "<Inrix docType=\"GetReferenceSchemaInGeography\" copyright=\"Copyright INRIX Inc.\" versionNumber=\"9.0\" createdDate=\"2016-10-04T08:28:37Z\" statusId=\"0\" statusText=\"\" responseId=\"0bee3ece-8ad3-4100-bd98-5bf4d6e7d6a5\">"
      "  <Dictionary xmlns=\"http://www.INRIX.com/LocDictionary\" xmlns:tdt=\"http://www.tisa.org/TPEG/TPEGDataTypes_2_0\" xmlns:olr=\"http://www.tisa.org/TPEG/OLR_1_0\">"
      "    <Report dictionaryUpdateDateTime=\"2016-09-15T03:40:51\" dictionaryVersion=\"16.2\">"
      "      <reportSegments>"
      "        <ReportSegmentID>8446643</ReportSegmentID>"
      "        <ReportSegmentLRC>"
      "          <method>"
      "            <olr:version>1.0</olr:version>"
      "            <olr:locationReference>"
      "              <olr:optionLinearLocationReference>"
      "                <olr:first>"
      "                  <olr:coordinate>"
      "                    <olr:longitude>1738792</olr:longitude>"
      "                    <olr:latitude>2577486</olr:latitude>"
      "                  </olr:coordinate>"
      "                  <olr:lineProperties>"
      "                    <olr:frc olr:table=\"olr001_FunctionalRoadClass\" olr:code=\"6\"/>"
      "                    <olr:fow olr:table=\"olr002_FormOfWay\" olr:code=\"2\"/>"
      "                    <olr:bearing>"
      "                      <olr:value>8</olr:value>"
      "                    </olr:bearing>"
      "                  </olr:lineProperties>"
      "                  <olr:pathProperties>"
      "                    <olr:lfrcnp olr:table=\"olr001_FunctionalRoadClass\" olr:code=\"6\"/>"
      "                    <olr:dnp>"
      "                      <olr:value>3572</olr:value>"
      "                    </olr:dnp>"
      "                    <olr:againstDrivingDirection>false</olr:againstDrivingDirection>"
      "                  </olr:pathProperties>"
      "                </olr:first>"
      "                <olr:last>"
      "                  <olr:coordinate>"
      "                    <olr:longitude>-1511</olr:longitude>"
      "                    <olr:latitude>2858</olr:latitude>"
      "                  </olr:coordinate>"
      "                  <olr:lineProperties>"
      "                    <olr:frc olr:table=\"olr001_FunctionalRoadClass\" olr:code=\"7\"/>"
      "                    <olr:fow olr:table=\"olr002_FormOfWay\" olr:code=\"3\"/>"
      "                    <olr:bearing>"
      "                      <olr:value>105</olr:value>"
      "                    </olr:bearing>"
      "                  </olr:lineProperties>"
      "                </olr:last>"
      "                <olr:positiveOffset>"
      "                  <olr:value>1637</olr:value>"
      "                </olr:positiveOffset>"
      "                <olr:negativeOffset>"
      "                  <olr:value>919</olr:value>"
      "                </olr:negativeOffset>"
      "              </olr:optionLinearLocationReference>"
      "            </olr:locationReference>"
      "          </method>"
      "        </ReportSegmentLRC>"
      "        <LinearConnectivity>"
      "          <negLink>"
      "            <ReportSegmentID>8446642</ReportSegmentID>"
      "          </negLink>"
      "          <posLink>"
      "            <ReportSegmentID>91840286</ReportSegmentID>"
      "          </posLink>"
      "        </LinearConnectivity>"
      "        <segmentLength>1018</segmentLength>"
      "        <segmentRefSpeed>0</segmentRefSpeed>"
      "     </reportSegments>"
      "  </Dictionary>"
      "</Inrix>";

  vector<openlr::LinearSegment> segments;
  pugi::xml_document doc;
  TEST_EQUAL(doc.load(openlrData), pugi::xml_parse_status::status_ok, ());
  TEST(openlr::ParseOpenlr(doc, segments), ());

  TEST_EQUAL(segments.size(), 1, ());

  auto const & segment = segments.front();
  TEST_EQUAL(segment.m_segmentId, 8446643, ());

  auto const locRef = segment.m_locationReference;
  TEST_EQUAL(locRef.m_points.size(), 2, ());

  auto const firstPoint = locRef.m_points.front();
  auto expectedLatLon = ms::LatLon{55.30683, 37.31041};
  TEST(firstPoint.m_latLon.EqualDxDy(expectedLatLon, 1e-5), (firstPoint.m_latLon, "!=", expectedLatLon));
  TEST_EQUAL(firstPoint.m_bearing, 8, ());
  TEST(firstPoint.m_formOfAWay == openlr::FormOfAWay::MULTIPLE_CARRIAGEWAY,
       ("Wrong form of a way."));
  TEST(firstPoint.m_functionalRoadClass == openlr::FunctionalRoadClass::FRC6,
       ("Wrong functional road class."));

  auto const secondPoint = locRef.m_points.back();
  expectedLatLon = ms::LatLon{55.33541, 37.29530};
  TEST(secondPoint.m_latLon.EqualDxDy(expectedLatLon, 1e-5), (secondPoint.m_latLon, "!=", expectedLatLon));
  TEST_EQUAL(secondPoint.m_bearing, 105, ());
  TEST(secondPoint.m_formOfAWay == openlr::FormOfAWay::SINGLE_CARRIAGEWAY, ("Wrong form of a way."));
  TEST(secondPoint.m_functionalRoadClass == openlr::FunctionalRoadClass::FRC7,
       ("Wrong functional road class."));
}

UNIT_TEST(LoadSamplePool_Test)
{
  platform::tests_support::ScopedFile sample(
      "sample.txt",
      "50670599\tRussia_Moscow Oblast_East-448-1=Russia_Moscow Oblast_East-448-2=Russia_Moscow Oblast_East-449-0\n"
      "50670604\tRussia_Moscow Oblast_East-3686-16=Russia_Moscow Oblast_East-3686-17=Russia_Moscow Oblast_East-3686-18=Russia_Moscow Oblast_East-3686-19\n"
      "50670916\tRussia_Moscow Oblast_East-3729-8=Russia_Moscow Oblast_East-3729-9=Russia_Moscow Oblast_East-3727-0=Russia_Moscow Oblast_East-3727-1=Russia_Moscow Oblast_East-3727-2\n"
      "50670943\tRussia_Moscow Oblast_East-24121-10=Russia_Moscow Oblast_East-24121-11=Russia_Moscow Oblast_East-24121-12\n"
      "50670944\tRussia_Moscow Oblast_East-24121-15=Russia_Moscow Oblast_East-24121-14\n"
      "50671025\tRussia_Moscow Oblast_East-25270-2=Russia_Moscow Oblast_East-25270-1=Russia_Moscow Oblast_East-25270-0\n"
      "50671086\tRussia_Moscow Oblast_East-25307-0=Russia_Moscow Oblast_East-25307-1\n");

  Index emptyIndex;  // Empty is ok for this test.
  auto const pool = LoadSamplePool(sample.GetFullPath(), emptyIndex);

  TEST_EQUAL(pool.size(), 7, ());

  TEST(pool[0].m_evaluation == openlr::ItemEvaluation::Unevaluated,
       ("pool[0].m_evaluation != openlr::ItemEvaluation::Unevaluated"));
  TEST_EQUAL(pool[0].m_partnerSegmentId.Get(), 50670599, ());
  TEST_EQUAL(pool[1].m_partnerSegmentId.Get(), 50670604, ());
  TEST_EQUAL(pool[2].m_partnerSegmentId.Get(), 50670916, ());

  TEST_EQUAL(pool[0].m_segments.size(), 3, ());
  TEST_EQUAL(pool[1].m_segments.size(), 4, ());
  TEST_EQUAL(pool[2].m_segments.size(), 5, ());

  TEST_EQUAL(pool[0].m_segments[0].m_segId, 1, ());
  TEST_EQUAL(pool[1].m_segments[0].m_segId, 16, ());
  TEST_EQUAL(pool[2].m_segments[0].m_segId, 8, ());
}
