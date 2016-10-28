#include "openlr/openlr_simple_decoder.hpp"

#include "routing/car_router.hpp"
#include "routing/road_graph_router.hpp"
#include "routing/router.hpp"

#include "storage/country_info_getter.hpp"

#include "indexer/classificator_loader.hpp"
#include "indexer/index.hpp"

#include "platform/local_country_file.hpp"
#include "platform/local_country_file_utils.hpp"
#include "platform/platform.hpp"

#include "coding/file_name_utils.hpp"

#include "std/iostream.hpp"

namespace
{
void LoadIndex(Index & index)
{
  vector<platform::LocalCountryFile> localFiles;

  string const dir = GetPlatform().WritableDir();
  auto const latestVersion = numeric_limits<uint64_t>::max();
  FindAllLocalMapsInDirectoryAndCleanup(dir, 0 /* version */, latestVersion, localFiles);

  Platform::TFilesWithType fwts;
  Platform::GetFilesByType(dir, Platform::FILE_TYPE_DIRECTORY, fwts);
  for (auto const & fwt : fwts)
  {
    string const & subdir = fwt.first;
    int64_t version;
    if (!platform::ParseVersion(subdir, version) || version > latestVersion)
      continue;

    string const fullPath = my::JoinFoldersToPath(dir, subdir);
    FindAllLocalMapsInDirectoryAndCleanup(fullPath, version, latestVersion, localFiles);
    Platform::EError err = Platform::RmDir(fullPath);
    if (err != Platform::ERR_OK && err != Platform::ERR_DIRECTORY_NOT_EMPTY)
      LOG(LWARNING, ("Can't remove directory:", fullPath, err));
  }


  // platform::FindAllLocalMapsInDirectoryAndCleanup(
  //     platform.WritableDir(), 0 /* version */,
  //     numeric_limits<uint64_t>::max() /* latestVersion */,
  //     localFiles);

  for (platform::LocalCountryFile & localFile : localFiles)
  {
    LOG(LINFO, ("Found mwm:", localFile));
    try
    {
      localFile.SyncWithDisk();
      index.RegisterMap(localFile);
    }
    catch (RootException const & ex)
    {
      CHECK(false, (ex.Msg(), "Bad mwm file:", localFile));
    }
  }
}
}  // namespace

using namespace openlr;

int main(int argc, char * argv[])
{
  if (argc != 2)
  {
    cout << "USAGE: " << argv[0] << " openlrdata" << endl;
    exit(0);
  }

  Index index;
  LoadIndex(index);

  classificator::Load();

  auto const infoGetter = storage::CountryInfoReader::CreateCountryInfoReader(GetPlatform());
  auto const countryFileGetter = [&infoGetter](m2::PointD const & pt)
  {
    return infoGetter->GetRegionCountryId(pt);
  };

  routing::CarRouter router(&index, countryFileGetter,
                            routing::CreateCarAStarBidirectionalRouter(index, countryFileGetter));

  OpenLRSimpleDecoder decoder(argv[1], index, router);
  decoder.Decode();

  return 0;
}
