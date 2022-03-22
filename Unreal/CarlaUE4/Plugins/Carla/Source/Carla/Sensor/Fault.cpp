#ifndef FAULT_H_
#define FAULT_H_

#include "Carla.h"

class FFault
{
protected:
  std::map<FString, FString> param;

public:
  virtual void apply(TArray<FColor> &) = 0;
virtual ~FFault() {}
  FFault(FString s=FString(TEXT("")))
  {
    parseParameters(s);
  }

private:
  struct cmpFString
  {
    bool operator()(const FString &a, const FString &b) const
    {
      return a.Equals(b,ESearchCase::IgnoreCase);
    }
  };

  void parseParameters(FString s)
  {

    TArray<FString> outarray;

    s.ParseIntoArrayLines(outarray, true);

    for (FString l : outarray)
    {
      FString le;
      FString ri;
      l.Split(FString(TEXT(":")), &le, &ri);
      this->param.insert(std::pair<FString, FString>(le, ri));

    }



  }
};

#endif // FAULT_H_
