#include "openlr/openlr_simple_decoder.hpp"

#include "routing/car_router.hpp"
#include "routing/road_graph_router.hpp"
#include "routing/router.hpp"
#include "routing/single_mwm_router.hpp"

#include "traffic/traffic_cache.hpp"

#include "storage/country_info_getter.hpp"

#include "indexer/classificator_loader.hpp"
#include "indexer/index.hpp"

#include "platform/local_country_file.hpp"
#include "platform/local_country_file_utils.hpp"
#include "platform/platform.hpp"

#include "coding/file_name_utils.hpp"

#include "std/cstdint.hpp"
#include "std/iostream.hpp"
#include "std/limits.hpp"

#include "3party/gflags/src/gflags/gflags.h"

DEFINE_string(input, "", "Path to OpenLR file.");
DEFINE_string(output, "output.txt", "Path to output file");
DEFINE_string(mwms_path, "", "Path to a folder with mwms.");
DEFINE_int32(limit, -1, "Max number of segments to handle. -1 for all.");
DEFINE_bool(multipoints_only, false, "Only segments with multiple points to handle.");
DEFINE_int32(num_threads, 1, "Number of threads.");

using namespace openlr;

namespace
{
const int32_t kMinNumThreads = 1;
const int32_t kMaxNumThreads = 128;

void LoadIndex(string const & pathToMWMFolder, Index & index)
{
  Platform::FilesList files;
  Platform::GetFilesByRegExp(pathToMWMFolder, string(".*\\") + DATA_FILE_EXTENSION, files);
  for (auto const & fileName : files)
  {

    auto const fullFileName = my::JoinFoldersToPath({pathToMWMFolder}, fileName);
    ModelReaderPtr reader(GetPlatform().GetReader(fullFileName, "f"));
    platform::LocalCountryFile localFile(pathToMWMFolder,
                                         platform::CountryFile(my::GetNameWithoutExtCopy(fileName)),
                                         version::ReadVersionDate(reader));
    LOG(LINFO, ("Found mwm:", fullFileName));
    try
    {
      localFile.SyncWithDisk();
      CHECK_EQUAL(index.RegisterMap(localFile).second, MwmSet::RegResult::Success,
                  ("Can't register mwm:", localFile));
    }
    catch (RootException const & ex)
    {
      CHECK(false, (ex.Msg(), "Bad mwm file:", localFile));
    }
  }
}

bool ValidateLimit(char const * flagname, int32_t value)
{
  if (value < -1)
  {
    printf("Invalid value for --%s: %d, must be greater or equal to -1\n", flagname,
           static_cast<int>(value));
    return false;
  }

  return true;
}

bool ValidateNumThreads(char const * flagname, int32_t value)
{
  if (value < kMinNumThreads || value > kMaxNumThreads)
  {
    printf("Invalid value for --%s: %d, must be between %d and %d inclusively\n", flagname,
           static_cast<int>(value), static_cast<int>(kMinNumThreads),
           static_cast<int>(kMaxNumThreads));
    return false;
  }

  return true;
}

bool ValidataMwmPath(char const * flagname, string const & value)
{
  if (value.empty())
  {
    printf("--%s should be specified\n", flagname);
    return false;
  }

  return true;
}

bool const g_limitDummy = google::RegisterFlagValidator(&FLAGS_limit, &ValidateLimit);
bool const g_numThreadsDummy =
    google::RegisterFlagValidator(&FLAGS_num_threads, &ValidateNumThreads);
bool const g_mwmsPathDummy = google::RegisterFlagValidator(&FLAGS_mwms_path, &ValidataMwmPath);
}  // namespace

int main(int argc, char * argv[])
{
  google::SetUsageMessage("OpenLR stats tool.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  classificator::Load();

  Index index;
  LoadIndex(FLAGS_mwms_path, index);

  auto const infoGetter = storage::CountryInfoReader::CreateCountryInfoReader(GetPlatform());
  auto const countryFileGetter = [&infoGetter](m2::PointD const & pt)
  {
    return infoGetter->GetRegionCountryId(pt);
  };

  traffic::TrafficCache trafficCache;

  routing::CarRouter router(index, countryFileGetter,
                            routing::SingleMwmRouter::CreateCarRouter(index, trafficCache));

  OpenLRSimpleDecoder decoder(FLAGS_input, index);
  decoder.Decode(FLAGS_output, FLAGS_limit, FLAGS_multipoints_only, FLAGS_num_threads);

  return 0;
}
