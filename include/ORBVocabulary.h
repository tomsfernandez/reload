


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "../deps/DBoW2/DBoW2/FORB.h"
#include "../deps/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{
/** Vocabulario que mapea descriptores ORB con Bow.
 * OBRVocabulary es un tipo basado en plantilla que define un tipo de descriptores y una clase de funciones DBOW2 para manipularlos.
 * OBRVocabulary usa Mat como descriptor (DBoW2::FORB::TDescriptor es Mat), y la clase DBoW2::FORB como implementación específica para ORB
 * de las funciones generales de manipulación de descriptores que requiere DBOW2.
 *
 * main.cc crea la única instancia de este objeto, carga su vocabulario desde un archivo,
 * y luego lo pasa como referencia a los constructores principales, iniciando la cascada.
 * En lo sucesivo, cada objeto que necesita un vocabulario lo recibe por referencia al construirse.
 *
 * Frame, KeyFrame, LoopClosing y Tracking lo denominan mpORBvocabulary.
 * KeyFrameDatabase lo denomina mpVoc.
 * Frame y KeyFrame utilizan solamente su método transform, que obtiene los BoW correspondientes a un conjunto de descriptores.
 * LoopClosing utiliza solamente su método score, que compara dos BoW.
 * KeyFrameDatabase utiliza score y size, este último simplemente informa la cantidad de palabras en el vocabulario.
 * Tracking se limita a pasarlo a los objetos que construye.  Actúa como pasamanos.
 *
 *
 *
 *
 */

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
