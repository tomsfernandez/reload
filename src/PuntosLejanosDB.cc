
#include "PuntosLejanosDB.h"

using namespace ORB_SLAM2;

PuntosLejanosDB::PuntosLejanosDB(const ORBVocabulary &voc):vocabularioBow(&voc)
{
	BowAPuntosLejanos.resize(voc.size());
}


