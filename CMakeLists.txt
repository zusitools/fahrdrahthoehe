cmake_minimum_required (VERSION 3.1) 
project (fahrdrahthoehe)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(parser)
generate_zusi_parser(zusi_parser ${CMAKE_CURRENT_BINARY_DIR}/zusi_parser
  WHITELIST
    Zusi::Strecke
    Zusi::Landschaft

    Dateiverknuepfung::Dateiname
    Face::i
    Landschaft::SubSet
    Landschaft::Verknuepfte
    Landschaft::lsb
    Material::TypLs3
    ModulDateiVerknuepfung::Datei
    NachfolgerSelbesModul::Nr
    NachfolgerAnderesModul::Datei
    StrElement::Anschluss
    StrElement::Drahthoehe
    StrElement::Fkt
    StrElement::NachGegen
    StrElement::NachNorm
    StrElement::NachGegenModul
    StrElement::NachNormModul
    StrElement::Ueberh
    StrElement::Volt
    StrElement::b
    StrElement::g
    Strecke::Datei
    Strecke::ModulDateien
    Strecke::StrElement
    Strecke::UTM
    SubSet::Face
    SubSet::MeshI
    SubSet::MeshV
    SubSet::Vertex
    UTM::UTM_NS
    UTM::UTM_WE
    UTM::UTM_Zone
    UTM::UTM_Zone2
    Verknuepfte::Datei
    Verknuepfte::p
    Verknuepfte::phi
    Vertex::U
    Vertex::U2
    Vertex::V
    Vertex::V2
    Vertex::n
    Vertex::p
  IGNORE_UNKNOWN
  USE_GLM)

add_executable(fahrdrahthoehe fahrdrahthoehe.cpp nowide/src/iostream.cpp)
target_link_libraries(fahrdrahthoehe PRIVATE zusi_parser)

find_package(Boost COMPONENTS filesystem REQUIRED)
target_include_directories(fahrdrahthoehe PRIVATE ${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR} nowide/include)
target_link_libraries(fahrdrahthoehe PRIVATE ${Boost_LIBRARIES})
target_compile_definitions(fahrdrahthoehe PRIVATE GLM_ENABLE_EXPERIMENTAL)
install(TARGETS fahrdrahthoehe DESTINATION bin)
