#include "zusi_parser/zusi_types.hpp"
#include "zusi_parser/zusi_parser.hpp"
#include "zusi_parser/utils.hpp"

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>

#include <rapidxml-1.13/rapidxml.hpp>
#include <rapidxml-1.13/rapidxml_print.hpp>

#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>
#include <boost/nowide/fstream.hpp>

#include <algorithm>
// #include <charconv>
#include <cstdlib>
#include <iomanip>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>

#if READ_LSB
#include "lsb.hpp"
#endif

#ifdef WIN32
#define UNICODE
#endif

struct Quad {
  std::array<Vec3, 4> v;
  Vec3 up; // normalized
  Vec3::value_type d; // base plane of the quad: ax+by+cz+d=0, where up == (a, b, c) is a normal vector of the plane
};

std::unordered_map<int, Quad> quad_by_element;
std::unordered_map<int, Vec3::value_type> hoehe_by_element;

std::map<int, std::map<int, std::vector<int>>> elements_by_coordinate_div_10;

std::unordered_set<std::string> kein_fahrdraht;  // LS3-Dateien ohne Fahrdraht-Subsets

std::ofstream dump;
// std::ofstream hoehen("hoehen.txt");

bool debug;

bool liesLs3(const std::string& dateiname, const std::string& rel, const glm::mat4& transform) {
  bool hat_fahrdraht = false;
  const auto& pfad = zusixml::zusiPfadZuOsPfad(dateiname, rel);
  if (kein_fahrdraht.find(pfad) != std::end(kein_fahrdraht)) {
    return false;
  }

  const auto& ls3 = zusixml::tryParse(pfad);
  if (!ls3 || !ls3->Landschaft) {
    return false;
  }

#if READ_LSB
  if (std::any_of(std::begin(ls3->Landschaft->children_SubSet), std::end(ls3->Landschaft->children_SubSet), [](const auto& s) { return s->TypLs3 == 14; })) {
    readLsb(ls3->Landschaft.get(), pfad);
  }
#endif

  // Fuer jedes Subset mit LS3-Typ "Fahrleitung": iteriere ueber Dreiecke
  for (const auto& subset : ls3->Landschaft->children_SubSet) {
    if (subset->TypLs3 != 14) {
      continue;
    }

#if !READ_LSB
    if (subset->MeshI != 0) {
      boost::nowide::cerr << "Warnung: " << pfad << " enthaelt Subset mit streng geheimen Geometriedaten (lsb-Format).\n";
    }
#endif

    if (subset->children_Vertex.size() == 0) {
      continue;
    }

    hat_fahrdraht = true;

    for (auto& vertex : subset->children_Vertex) {
      vertex.p = glm::vec3(transform * glm::vec4(vertex.p.x, vertex.p.y, vertex.p.z, 1));
    }

    // Performance: Triff Vorauswahl von Streckenelementen anhand von x- und y-Koordinaten
    const auto [ minXIt, maxXIt ] = std::minmax_element(std::begin(subset->children_Vertex), std::end(subset->children_Vertex),
        [](const auto& lhs, const auto& rhs) { return lhs.p.x < rhs.p.x; });
    const auto [ minYIt, maxYIt ] = std::minmax_element(std::begin(subset->children_Vertex), std::end(subset->children_Vertex),
        [](const auto& lhs, const auto& rhs) { return lhs.p.y < rhs.p.y; });

    const auto minX = minXIt->p.x;
    const auto maxX = maxXIt->p.x;
    assert(minX <= maxX);

    const auto minY = minYIt->p.y;
    const auto maxY = maxYIt->p.y;
    assert(minY <= maxY);

    std::vector<std::pair<int, Quad*>> quads_to_check {};
#ifndef NDEBUG
    std::vector<int> checkedElements {};
#endif

    auto it_x = elements_by_coordinate_div_10.lower_bound(minX / 10 - 1);
    const auto it_x_end = elements_by_coordinate_div_10.upper_bound(maxX / 10 + 2);
    for (; it_x != it_x_end; ++it_x) {
      auto it_y = it_x->second.lower_bound(minY / 10 - 1);
      const auto it_y_end = it_x->second.upper_bound(maxY / 10 + 2);
      for (; it_y != it_y_end; ++it_y) {
        for (const auto elementNr : it_y->second) {
          quads_to_check.push_back(std::make_pair(elementNr, &quad_by_element.at(elementNr)));
#ifndef NDEBUG
          checkedElements.push_back(elementNr);
#endif
        }
      }
    }

    for (const auto& face : subset->children_Face) {
      // Fuer jede Dreiecksseite berechne Strecke zwischen Vertices
      for (const auto& vertices : {
          // TODO: Nur annaehernd horizontale Faces betrachten
          std::make_pair(&subset->children_Vertex[face.i[0]], &subset->children_Vertex[face.i[1]]),
          std::make_pair(&subset->children_Vertex[face.i[1]], &subset->children_Vertex[face.i[2]]),
          std::make_pair(&subset->children_Vertex[face.i[2]], &subset->children_Vertex[face.i[0]]) }) {
        Vec3 richtungsvektor = vertices.second->p - vertices.first->p;

        // Fuer jedes Streckenelement berechne Schnittpunkt der Strecke mit dessen Ebene
#ifndef NDEBUG
        for (const auto& [ elementNr, quad_ ] : quad_by_element) {
          const auto* quad = &quad_;
#else
        for (const auto& [ elementNr, quad ] : quads_to_check) {
#endif
          // quad->v = { unten rechts, unten links, oben rechts, oben links }
          for (const auto& [ v1, v2, v3 ] : { std::forward_as_tuple(quad->v[0], quad->v[2], quad->v[1]), std::forward_as_tuple(quad->v[1], quad->v[3], quad->v[2]) }) {
            Vec3 pos_bary;
            bool intersect = glm::intersectLineTriangle(vertices.first->p, richtungsvektor, v1, v2, v3, pos_bary);
            // intersectRayTriangle: Z-Koordinate von pos_bary ist Faktor fuer Richtungsvektor
            // intersectLineTriangle: X-Koordinate von pos_bary ist Faktor fuer Richtungsvektor
            Vec3::value_type faktor = pos_bary.x;
            if (intersect && faktor >= -0.001 && faktor <= 1.001) {
#ifndef NDEBUG
              if (std::find(std::begin(checkedElements), std::end(checkedElements), elementNr) == std::end(checkedElements)) {
                assert(false);
              }
#endif

              Vec3 schnittpunkt = vertices.first->p + faktor * richtungsvektor;
              if (debug) {
                dump << "<Ankerpunkt><p X='" << schnittpunkt.x << "' Y='" << schnittpunkt.y << "' Z='" << schnittpunkt.z << "'/></Ankerpunkt>\n";
              }

              Vec3::value_type dist = glm::dot(quad->up, schnittpunkt) + quad->d;
              hoehe_by_element[elementNr] = std::min(hoehe_by_element[elementNr], dist);
            }
          }
        }
      }
    }
  }

  for (const auto& verknuepfte : ls3->Landschaft->children_Verknuepfte) {
    glm::mat4 rot_verkn = glm::eulerAngleXYZ(verknuepfte->phi.x, verknuepfte->phi.y, verknuepfte->phi.z);
    glm::mat4 transl_verkn = glm::translate(glm::mat4(), glm::vec3(verknuepfte->p.x, verknuepfte->p.y, verknuepfte->p.z));
    glm::mat4 transform_verkn = transl_verkn * rot_verkn * transform;

    hat_fahrdraht = liesLs3(verknuepfte->Datei.Dateiname, pfad, transform_verkn) || hat_fahrdraht; // Reihenfolge!
  }

  if (!hat_fahrdraht) {
    kein_fahrdraht.emplace(pfad);
  }

  return hat_fahrdraht;
}

void schreibeSt3(const std::string_view dateiname) {
  rapidxml::xml_document<> doc;
  zusixml::FileReader reader(dateiname);
  doc.parse<rapidxml::parse_non_destructive>(const_cast<char*>(reader.data()));

  rapidxml::xml_node<>* zusi_node = doc.first_node("Zusi");
  rapidxml::xml_node<>* strecke_node = zusi_node->first_node("Strecke");
  for (auto* str_element_node = strecke_node->first_node("StrElement"); str_element_node; str_element_node = str_element_node->next_sibling("StrElement")) {
    rapidxml::xml_attribute<>* nr_attrib = str_element_node->first_attribute("Nr");
    if (!nr_attrib) {
      continue;
    }

#if 0 // don't have <charconv> yet
    int nr;
    if (!std::from_chars(nr_attrib->value(), nr_attrib->value() + nr_attrib->value_size(), nr)) {
      continue;
    }
#else
    const std::string s { nr_attrib->value(), nr_attrib->value_size() };
    int nr = atoi(s.c_str());
#endif

    auto it = hoehe_by_element.find(nr);
    if (it == std::end(hoehe_by_element)) {
      continue;
    }

    if (it->second == std::numeric_limits<Vec3::value_type>::infinity()) {
      continue;
    }

    auto val_as_string = std::to_string(it->second);
    auto* newval = doc.allocate_string(val_as_string.c_str());

    rapidxml::xml_attribute<>* drahthoehe_attrib = str_element_node->first_attribute("Drahthoehe");
    if (drahthoehe_attrib) {
      drahthoehe_attrib->value(newval);
    } else {
      str_element_node->append_attribute(doc.allocate_attribute("Drahthoehe", newval));
    }
  }

  std::string out_string;
  rapidxml::print(std::back_inserter(out_string), doc, rapidxml::print_no_indenting);

  std::ofstream o(std::string(dateiname) + ".new.st3", std::ios::binary);
  o << out_string;
}

std::vector<std::pair<int, int>> segmentiere(std::unordered_set<int> elemente, const Strecke& strecke) {
  std::function<int(int, bool)> findeWeitestenNachfolger = [&strecke, &elemente, &findeWeitestenNachfolger](int elementNr, bool norm) -> int {
    if (elementNr < 0 || static_cast<std::size_t>(elementNr) >= strecke.children_StrElement.size()) {
      return elementNr;
    }

    const auto& strElement = strecke.children_StrElement.at(elementNr);
    if (!strElement) {
      return elementNr;
    }

    // TODO: Annahme: Wenn der erste Nachfolger ein NachXXXModul ist, folgen danach keine NachXXX-Elemente
    const auto& nachfolger = norm ? strElement->children_NachNorm : strElement->children_NachGegen;
    if (nachfolger.size() == 0) {
      return elementNr;
    }

    int nachfolgerNr = nachfolger.front().Nr;

    auto it = elemente.find(nachfolgerNr);
    if (it == std::end(elemente)) {
      return elementNr;
    }

    if (nachfolgerNr < 0 || static_cast<std::size_t>(nachfolgerNr) >= strecke.children_StrElement.size()) {
      return elementNr;
    }

    bool nachfolgerNorm = ((strElement->Anschluss >> (norm ? 0 : 8)) & 1) == 0;
    const auto& nachfolgerElement = strecke.children_StrElement.at(nachfolgerNr);
    if (!nachfolgerElement) {
      return elementNr;
    }

    const auto& nachfolgerVorgaenger = nachfolgerNorm ? nachfolgerElement->children_NachGegen : nachfolgerElement->children_NachNorm;
    if (nachfolgerVorgaenger.size() == 0 || nachfolgerVorgaenger.front().Nr != elementNr) {
      return elementNr;
    }

    elemente.erase(it);
    return findeWeitestenNachfolger(nachfolgerNr, nachfolgerNorm);
  };

  std::vector<std::pair<int, int>> result;

  while (elemente.size() > 0) {
    auto it = std::begin(elemente);
    int elementNr = *it;
    elemente.erase(it);

    result.emplace_back(std::minmax(findeWeitestenNachfolger(elementNr, true), findeWeitestenNachfolger(elementNr, false)));
  }

  return result;
}

int main(int argc, char** argv) {
  [[maybe_unused]] boost::nowide::args a(argc, argv);

  if (argc < 2) {
    boost::nowide::cerr << "Usage: ./fahrdrahthoehe [--debug] [st3 file]\n";
    return 1;
  }

  for (size_t i = 1; i < argc - 1; ++i) {
    using namespace std::literals::string_literals;
    if ("--debug"s == argv[i]) {
      debug = true;
    } else {
      boost::nowide::cerr << "Usage: ./fahrdrahthoehe [--debug] [st3 file]\n";
      return 1;
    }
  }

  if (debug) {
    dump.open("debug.ls3");
  }

  // Lies Moduldatei ein
  const auto& st3 = zusixml::tryParse(argv[argc-1]);
  if (!st3 || !st3->Strecke) {
    return 1;
  }

  if (debug) {
    dump << "<Zusi><Landschaft>";
  }

  // Fuer jedes Streckenelement, das elektrifiziert ist, bestimme Schnittbereich.
  // Dieser ist ein Viereck (= 2 Dreiecke), dessen Normalenvektor der Richtungsvektor des Streckenelements ist.
  // Seine Breite ist die Breite des Stromabnehmer-Arbeitsbereiches.
  // Seine Hoehe ist die maximale Stromabnehmerhoehe.
  for (const auto& element : st3->Strecke->children_StrElement) {
    if (!element || (element->Volt == 0) || (element->Fkt & 0x2 /* Keine Gleisfunktion */) != 0) {
      continue;
    }

    constexpr Vec3::value_type arbeitsbreite_sa = /* +/- */ 0.8; // pro Richtung
    constexpr Vec3::value_type max_sa_hoehe = 7.0; // max. 6.5m + Puffer

    Vec3 normalenvektor = element->b - element->g;
    Vec3 ortsvektor = element->g + Vec3::value_type(0.5) * normalenvektor;

    elements_by_coordinate_div_10[ortsvektor[0] / 10][ortsvektor[1] / 10].push_back(element->Nr);

    Vec3 up = glm::rotate(Vec3(0, 0, 1), static_cast<Vec3::value_type>(element->Ueberh), normalenvektor);

    Vec3 v = glm::normalize(glm::cross(up, normalenvektor));
    Vec3 v1 = ortsvektor + arbeitsbreite_sa * v;
    Vec3 v2 = ortsvektor - arbeitsbreite_sa * v;
    Vec3 v3 = v1 + max_sa_hoehe * up;
    Vec3 v4 = v2 + max_sa_hoehe * up;

    if (debug) {
      dump << "<SubSet>" <<
        "<Vertex><p X='" << v1[0] << "' Y='" << v1[1] << "' Z='" << v1[2] << "'/></Vertex>" <<
        "<Vertex><p X='" << v2[0] << "' Y='" << v2[1] << "' Z='" << v2[2] << "'/></Vertex>" <<
        "<Vertex><p X='" << v3[0] << "' Y='" << v3[1] << "' Z='" << v3[2] << "'/></Vertex>" <<
        "<Vertex><p X='" << v4[0] << "' Y='" << v4[1] << "' Z='" << v4[2] << "'/></Vertex>" <<
        "<Face i='0;1;2'/><Face i='3;2;1'/>" <<
        "<Face i='2;1;0'/><Face i='1;2;3'/>" <<
        "</SubSet>\n";
    }

    quad_by_element.emplace(std::make_pair(element->Nr, Quad {
          { v1, v2, v3, v4 },
          up,
          -glm::dot(up, ortsvektor)
    }));
    hoehe_by_element.emplace(std::make_pair(element->Nr, std::numeric_limits<Vec3::value_type>::infinity()));
  }

  // Lies Landschaftsdatei ein (rekursiv)
  std::string rel;
  glm::mat4 transform;
  liesLs3(st3->Strecke->Datei.Dateiname, rel, transform);

  // Lies Landschaft von Nachbarmodulen ein
  for (const auto& nachbarmodul_name : st3->Strecke->children_ModulDateien) {
    const auto& nachbarmodul_pfad = zusixml::zusiPfadZuOsPfad(nachbarmodul_name->Datei.Dateiname, argv[1]);
    const auto& nachbarmodul_st3 = zusixml::tryParse(nachbarmodul_pfad);
    if (!nachbarmodul_st3 || !nachbarmodul_st3->Strecke) {
      boost::nowide::cerr << "Nachbarmodul " << nachbarmodul_pfad << ": nicht gefunden\n";
      continue;
    }
    if ((nachbarmodul_st3->Strecke->UTM->UTM_Zone != st3->Strecke->UTM->UTM_Zone) || (nachbarmodul_st3->Strecke->UTM->UTM_Zone2 != st3->Strecke->UTM->UTM_Zone2)) {
      boost::nowide::cerr << "Nachbarmodul " << nachbarmodul_pfad << ": verschiedene UTM-Zonen\n";
      continue;
    }
    auto utm_transform = glm::vec3((nachbarmodul_st3->Strecke->UTM->UTM_WE - st3->Strecke->UTM->UTM_WE) * 1000, (nachbarmodul_st3->Strecke->UTM->UTM_NS - st3->Strecke->UTM->UTM_NS) * 1000, 0);
    transform = glm::translate(glm::mat4(), utm_transform);
    liesLs3(nachbarmodul_st3->Strecke->Datei.Dateiname, rel, transform);
  }

  if (debug) {
    dump << "</Landschaft></Zusi>";
  }

  std::unordered_set<int> kein_fahrdraht_warnung {};

  constexpr Vec3::value_type diff_warning_threshold = 0.1;
  size_t anzahl_korrekturen_kleiner_threshold = 0;

  // Bestimme Hoehen
  for (const auto& element : st3->Strecke->children_StrElement) {
    if (!element) {
      continue;
    }

    auto it = hoehe_by_element.find(element->Nr);
    if (it == std::end(hoehe_by_element)) {
      continue;
    }
    if (it->second == std::numeric_limits<Vec3::value_type>::infinity()) {
      kein_fahrdraht_warnung.emplace(element->Nr);
    } else {
      auto diff = it->second - element->Drahthoehe;
      if (std::abs(diff) >= diff_warning_threshold) {
        boost::nowide::cerr << "Korrektur um " << std::showpos << (diff * 100) << std::noshowpos << " cm an Element " << element->Nr << "\n";
      } else {
        ++anzahl_korrekturen_kleiner_threshold;
      }
    }
  }

  if (anzahl_korrekturen_kleiner_threshold > 0) {
    boost::nowide::cerr << anzahl_korrekturen_kleiner_threshold << " Korrektur(en) um < 10 cm\n";
  }

  auto kein_fahrdraht_warnung_segmentiert = segmentiere(kein_fahrdraht_warnung, *st3->Strecke.get());
  std::sort(std::begin(kein_fahrdraht_warnung_segmentiert), std::end(kein_fahrdraht_warnung_segmentiert));
  for (const auto& [ anfang, ende ] : kein_fahrdraht_warnung_segmentiert) {
    if (anfang == ende) {
      boost::nowide::cerr << "Warnung: Keine Hoehe fuer elektrifiziertes Element " << anfang << " bestimmt\n";
    } else {
      boost::nowide::cerr << "Warnung: Keine Hoehe fuer elektrifizierte Elemente " << anfang << "-" << ende << " bestimmt\n";
    }
  }

  schreibeSt3(argv[argc-1]);
}
