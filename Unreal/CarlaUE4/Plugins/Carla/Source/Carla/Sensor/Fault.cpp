#include "Fault.h"

struct cmpFString
{
  bool operator()(const FString &a, const FString &b) const
  {
    return a.Equals(b, ESearchCase::IgnoreCase);
  }
};

void FFault::parseParameters(FString s)
{

  TArray<FString> outarray;

  s.ParseIntoArrayLines(outarray, true);

  for (FString l : outarray)
  {
    FString le;
    FString ri;
    l.Split(FString(TEXT(":")), &le, &ri);
    param.insert(std::pair<FString, FString>(le, ri));
  }
}
