#include "zusi_parser/zusi_types.hpp"
#include "zusi_parser/zusi_parser.hpp"
#include "zusi_parser/utils.hpp"

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>

struct Quad {
  std::array<Vec3, 4> v;
  Vec3 up; // normalized
  Vec3::value_type d; // base plane of the quad: ax+by+cz+d=0, where up == (a, b, c) is a normal vector of the plane
};

std::unordered_map<int, Quad> quad_by_element;
std::map<int, Vec3::value_type> hoehe_by_element;

std::unordered_set<std::string> kein_fahrdraht;  // LS3-Dateien ohne Fahrdraht-Subsets

std::ofstream dump("debug.ls3");
// std::ofstream hoehen("hoehen.txt");

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

  // Fuer jedes Subset mit LS3-Typ "Fahrleitung": iteriere ueber Dreiecke
  for (const auto& subset : ls3->Landschaft->children_SubSet) {
    if (subset->TypLs3 != 14) {
      continue;
    }

    if (subset->MeshI != 0) {
      std::cerr << "Fehler: " << pfad << " enthaelt Subset mit streng geheimen Geometriedaten (lsb-Format).";
      exit(1);
    }

    hat_fahrdraht = true;

    for (auto& vertex : subset->children_Vertex) {
      vertex->p = glm::vec3(transform * glm::vec4(vertex->p.x, vertex->p.y, vertex->p.z, 1));
    }

    for (const auto& face : subset->children_Face) {
      // Fuer jede Dreiecksseite berechne Strecke zwischen Vertices
      for (const auto& vertices : {
          // TODO: Vertices und Faces im Parser inlinen
          // TODO: Nur annaehernd horizontale Faces betrachten
          std::make_pair(subset->children_Vertex[face->i[0]].get(), subset->children_Vertex[face->i[1]].get()),
          std::make_pair(subset->children_Vertex[face->i[1]].get(), subset->children_Vertex[face->i[2]].get()),
          std::make_pair(subset->children_Vertex[face->i[2]].get(), subset->children_Vertex[face->i[0]].get()) }) {
        Vec3 richtungsvektor = vertices.second->p - vertices.first->p;

        // Fuer jedes Streckenelement berechne Schnittpunkt der Strecke mit dessen Ebene
        for (const auto& [ elementNr, quad ] : quad_by_element) {
          // quad.v = { unten rechts, unten links, oben rechts, oben links }
          for (const auto& [ v1, v2, v3 ] : { std::forward_as_tuple(quad.v[0], quad.v[2], quad.v[1]), std::forward_as_tuple(quad.v[1], quad.v[3], quad.v[2]) }) {
            Vec3 pos_bary;
            bool intersect = glm::intersectLineTriangle(vertices.first->p, richtungsvektor, v1, v2, v3, pos_bary);
            // intersectRayTriangle: Z-Koordinate von pos_bary ist Faktor fuer Richtungsvektor
            // intersectLineTriangle: X-Koordinate von pos_bary ist Faktor fuer Richtungsvektor
            Vec3::value_type faktor = pos_bary.x;
            if (intersect && faktor >= -0.001 && faktor <= 1.001) {

              Vec3 schnittpunkt = vertices.first->p + faktor * richtungsvektor;
              dump << "<Ankerpunkt><p X='" << schnittpunkt.x << "' Y='" << schnittpunkt.y << "' Z='" << schnittpunkt.z << "'/></Ankerpunkt>\n";

              Vec3::value_type dist = glm::dot(quad.up, schnittpunkt) + quad.d;
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

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./fahrdrahthoehe [st3 file]\n";
    return 1;
  }

  // Lies Moduldatei ein
  const auto& st3 = zusixml::tryParse(argv[1]);
  if (!st3 || !st3->Strecke) {
    return 1;
  }

  dump << "<Zusi><Landschaft>";

  // Fuer jedes Streckenelement, das elektrifiziert ist, bestimme Schnittbereich.
  // Dieser ist ein Viereck (= 2 Dreiecke), dessen Normalenvektor der Richtungsvektor des Streckenelements ist.
  // Seine Breite ist die Breite des Stromabnehmer-Arbeitsbereiches.
  // Seine Hoehe ist die maximale Stromabnehmerhoehe.
  for (const auto& element : st3->Strecke->children_StrElement) {
    if (element->Volt == 0) {
      continue;
    }

    constexpr Vec3::value_type arbeitsbreite_sa = /* +/- */ 0.8; // pro Richtung
    constexpr Vec3::value_type max_sa_hoehe = 7.0; // max. 6.5m + Puffer

    Vec3 normalenvektor = element->b - element->g;
    Vec3 ortsvektor = element->g + 0.5 * normalenvektor;

    Vec3 up = glm::rotate(Vec3(0, 0, 1), static_cast<Vec3::value_type>(element->Ueberh), normalenvektor);

    Vec3 v = glm::normalize(glm::cross(up, normalenvektor));
    Vec3 v1 = ortsvektor + arbeitsbreite_sa * v;
    Vec3 v2 = ortsvektor - arbeitsbreite_sa * v;
    Vec3 v3 = v1 + max_sa_hoehe * up;
    Vec3 v4 = v2 + max_sa_hoehe * up;

    dump << "<SubSet>" <<
      "<Vertex><p X='" << v1[0] << "' Y='" << v1[1] << "' Z='" << v1[2] << "'/></Vertex>" <<
      "<Vertex><p X='" << v2[0] << "' Y='" << v2[1] << "' Z='" << v2[2] << "'/></Vertex>" <<
      "<Vertex><p X='" << v3[0] << "' Y='" << v3[1] << "' Z='" << v3[2] << "'/></Vertex>" <<
      "<Vertex><p X='" << v4[0] << "' Y='" << v4[1] << "' Z='" << v4[2] << "'/></Vertex>" <<
      "<Face i='0;1;2'/><Face i='3;2;1'/>" <<
      "<Face i='2;1;0'/><Face i='1;2;3'/>" <<
      "</SubSet>\n";

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
  dump << "</Landschaft></Zusi>";

  // Bestimme Hoehen
  for (const auto& element : st3->Strecke->children_StrElement) {
    auto it = hoehe_by_element.find(element->Nr);
    if (it == std::end(hoehe_by_element)) {
      continue;
    }
    if (it->second == std::numeric_limits<Vec3::value_type>::infinity()) {
      std::cerr << "Warnung: Keine Hoehe fuer elektrifiziertes Element " << it->first << " bestimmt\n";
    }

    element->Drahthoehe = it->second;
    element->Drahthoehe_str = std::to_string(element->Drahthoehe);
  }

  // ¡Daten rausschreiben!
  for (const auto& [ elemNr, hoehe ] : hoehe_by_element) {
    std::cout << elemNr << " " << hoehe << "\n";
  }

  // st3->dump(std::cout);
}
