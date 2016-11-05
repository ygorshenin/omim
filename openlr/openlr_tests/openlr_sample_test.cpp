#include "testing/testing.hpp"

#include "openlr/openlr_sample.hpp"

#include "indexer/index.hpp"

#include "platform/platform_tests_support/scoped_file.hpp"

using namespace openlr;

UNIT_TEST(LoadS_Test)
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

  TEST(!pool[0].m_evaluation, ());
  TEST_EQUAL(pool[0].m_parterSegmentId.Get(), 50670599, ());
  TEST_EQUAL(pool[1].m_parterSegmentId.Get(), 50670604, ());
  TEST_EQUAL(pool[2].m_parterSegmentId.Get(), 50670916, ());

  TEST_EQUAL(pool[0].m_segments.size(), 3, ());
  TEST_EQUAL(pool[1].m_segments.size(), 4, ());
  TEST_EQUAL(pool[2].m_segments.size(), 5, ());

  TEST_EQUAL(pool[0].m_segments[0].m_segId, 1, ());
  TEST_EQUAL(pool[1].m_segments[0].m_segId, 16, ());
  TEST_EQUAL(pool[2].m_segments[0].m_segId, 8, ());
}
