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

#include "std/iostream.hpp"
#include "std/limits.hpp"

#include "3party/gflags/src/gflags/gflags.h"

DEFINE_string(olr_data_path, "", "Path to OpenLR file.");
DEFINE_int32(limit, openlr::OpenLRSimpleDecoder::kHandleAllSegmets,
             "Max number of segments to handle. -1 for all.");
DEFINE_bool(multipoints_only, false, "Only segments with multiple points to handle.");

using namespace openlr;

namespace
{
void LoadIndex(Index & index)
{
  vector<platform::LocalCountryFile> localFiles;

  auto const latestVersion = numeric_limits<int64_t>::max();
  FindAllLocalMapsAndCleanup(latestVersion, localFiles);

  for (platform::LocalCountryFile & localFile : localFiles)
  {
    LOG(LINFO, ("Found mwm:", localFile));
    try
    {
      localFile.SyncWithDisk();
      auto const result = index.RegisterMap(localFile);
      CHECK_EQUAL(result.second, MwmSet::RegResult::Success, ("Can't register mwm:", localFile));
    }
    catch (RootException const & ex)
    {
      CHECK(false, (ex.Msg(), "Bad mwm file:", localFile));
    }
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  google::SetUsageMessage("OpenLR stats tool.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  classificator::Load();

  Index index;
  LoadIndex(index);

  auto const infoGetter = storage::CountryInfoReader::CreateCountryInfoReader(GetPlatform());
  auto const countryFileGetter = [&infoGetter](m2::PointD const & pt)
  {
    return infoGetter->GetRegionCountryId(pt);
  };

  traffic::TrafficCache trafficCache;

  routing::CarRouter router(index, countryFileGetter,
                            routing::SingleMwmRouter::CreateCarRouter(index, trafficCache));

  OpenLRSimpleDecoder decoder(FLAGS_olr_data_path, index);
  decoder.Decode(FLAGS_limit, FLAGS_multipoints_only);

  return 0;
}
