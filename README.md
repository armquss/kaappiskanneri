# Kaappiskanneri

Osoita puhelimen kameralla kaapin ovea, niin sovellus kertoo, mitä kaapissa on. Kaikki toimii
puhelimessa ilman palvelinta ja ilman verkkoyhteyttä (offline-PWA).

## Miten tunnistus toimii

Sovellus ei yritä tunnistaa pelkkää ovea. Se tunnistaa **koko ympäristön**: laatat, kahvat, kodinkoneet,
ikkunat ja tasot. Näin myös täysin samannäköiset ovet erottuvat toisistaan.

1. **Kartoitus.** Kuvaat kaappiseinän kerran kauempaa ja merkitset ovet kuvaan.
2. **Tunnistus.** Kamerakuvasta lasketaan ORB-piirteet, ja niitä verrataan kartoituskuviin.
   RANSAC-homografia kertoo, mitä kohtaa seinästä kamera katsoo ja mistä kulmasta.
3. **Tähtäin.** Ruudun keskipiste projisoidaan kartoituskuvaan. Se ovialue, johon tähtäin osuu,
   on valittu kaappi.
4. **Seuranta.** Optinen vuo (Lucas–Kanade) liikkeen ennustuksen kanssa pyörii jokaisella kuvalla
   (~10 ms). Siksi kaappi pysyy tiedossa myös silloin, kun tulet niin lähelle, että kuvassa näkyy
   pelkkää ovea.
5. **Ohjattu haku ja palautus.** Kun sijainti tiedetään suunnilleen, vastaavuuksia haetaan vain sen
   läheltä, joten naapuriovi ei sekoita. Jos seuranta hetkeksi katkeaa, sijainti palautetaan
   viimeisimmästä hyvästä kamerakuvasta.

Konenäkö pyörii kahdessa erillisessä säikeessä (Web Worker, OpenCV.js). **Seurantasäie** käsittelee
jokaisen kamerakuvan (~10–40 ms) ja pitää kirjaa kuvien välisestä liikkeestä. **Tunnistussäie**
ankkuroi sijainnin kartoituskuvaan taustalla. Koska tunnistus voi puhelimessa kestää sekunnin,
sen tulos siirretään liikehistorian avulla siihen kuvaan, joka on ruudulla nyt, joten seuranta ei
pysähdy koskaan tunnistuksen ajaksi. Lähikuvissa tunnistus vertaa kuvaa myös aiemmin varmasti
ankkuroituihin *avainkuviin*, mutta vain ennustetun sijainnin läheltä, joten identtinen naapuriovi
ei voi sekoittaa. Kamerakuva luetaan suoraan kameravirrasta kuvakehyksinä (WebCodecs), ja ruudulle piirretään juuri se
kehys, josta sijainti laskettiin, yhdessä sen rajojen kanssa. Näin rajat eivät jää liikkeessä kuvan jälkeen.

Kun sijainti on hukassa, kartoituskuvaa verrataan neljässä eri mittakaavassa. Näin tunnistus toimii
myös silloin, kun koko seinä on kuvassa ja kaapit näkyvät kamerassa moninkertaisesti pienempinä kuin
kartoituskuvassa. Haku aloitetaan viimeksi käytetystä kartoituskuvasta, se lopetetaan vahvaan osumaan,
löydetty sijainti tarkennetaan ohjatulla haulla, ja liike-epäterävät kuvat ohitetaan.

### Oppiva kartoitus

Käytön aikana sovellus tallentaa laitteelle varmasti tunnistettuja kamerakuvia (vain piirteet, ei
kuvia; noin 30 kt kappale, enintään 100 per kartoituskuva). Ne lajitellaan paikan, etäisyyden ja
valaistuksen mukaan, ja kustakin lokerosta säilytetään paras. Seuraavalla käynnistyksellä tunnistus
löytää kaapin näistä nopeammin ja myös olosuhteissa, joissa kartoituskuva ei riitä, esimerkiksi
iltavalossa (testissä 8/40 → 34/40). Ilman ennakkotietoa käytetään vain kuvia, joissa näkyy muutakin
kuin yksi ovi. Lisäksi osuman pitää nojata myös ovien ulkopuolisiin piirteisiin, jotta identtiset
ovet eivät sekoitu. Uusi avainkuva on ensin väliaikainen: se tallennetaan vasta, kun ovien reunat
vahvistavat sijainnin, ja sijainti korjataan reunojen mukaan. Avainkuva, joka toistuvasti on
ristiriidassa kartoituskuvan kanssa, poistetaan. Opitut voi unohtaa asetuksista.

### Ovien reunojen kohdistus

Kun merkitset oven, sovellus siirtää rajan sivut oven todellisiin reunoihin. Jokaiselle sivulle
haetaan ehdokassuoria, ja yhdistelmä valitaan reunan vahvuuden, muodon ja reunan kummankin puolen
pinnan perusteella. Näin naapurioven reuna tai oven sisäpaneelin ura ei voita. Testissä kulmien
keskivirhe pieneni 19–25 pikselistä noin 3,5 pikseliin. Kohdistuksen voi kumota, ja valitun oven
voi kohdistaa uudelleen Kohdista-napista.

Skannatessa samaa tehdään jatkuvasti: ennustetun oven sivut kohdistetaan kamerakuvan reunoihin, ja
sijaintia korjataan niiden mukaan vähitellen. Kun reunat juuri vahvistivat sijainnin, niiden kanssa
ristiriitainen tunnistus hylätään. Valkoisissa, piirteettömissä ovissa reunat ovat lähikuvissa
luotettavin tieto oven paikasta.

### Lähikuvat ja syvyys

Lähellä ja vinosta katsottaessa huonekalut, katto ja syvennykset liikkuvat eri tavalla kuin
oviseinä (parallaksi). Siksi tunnistuksen tulos sovitetaan uudelleen pelkkiin ovialueen piirteisiin,
jos ne sopivat siihen selvästi paremmin. Tasaisessa näkymässä käytetään koko kuvaa, koska se on
silloin tarkempi. Jos tunnistus ei toistuvasti vahvista seurannan sijaintia, sijaintia epäillään ja
koko kuvaa verrataan uudelleen. Hukassa ollessa laajalla haulla löytynyt osuma hyväksytään vain, jos
se kattaa riittävän alueen kuvasta, koska laatat ja listat voisivat muuten antaa siirtyneen osuman.
Jos tähtäin osuu kartoitetun alueen ulkopuolelle, sijainti säilyy, kunhan kaapit näkyvät ruudulla.

Testissä, jossa todellisesta kartoituskuvasta renderöitiin syvyyskerroksia (kolme reittiä, 360 kuvaa),
kulmavirheen mediaani pieneni 18,8 pikselistä 14,1 pikseliin ja 90 %:n raja 80 pikselistä 60
pikseliin. Aivan lähellä parannus on pieni (43 → 41 px), sillä tunnistus nojaa kartoituskuvaan,
joka on otettu kauempaa.

Kevyempi käsittely: kameran kirkkauskanava pienennetään suoraan, tunnistusta ajetaan harvemmin, kun
reunat vahvistavat sijainnin, ja seuranta jatkuu tunnistuksen aikana täydellä nopeudella.

## Julkaisu GitHub Pagesiin

1. Luo repo (esim. `kaappiskanneri`) ja lataa tämän kansion sisältö sen juureen.
2. Valitse Settings → Pages → *Deploy from a branch*, `main` / `(root)`.
3. Avaa `https://<käyttäjä>.github.io/kaappiskanneri/` puhelimen Chromella ja valitse
   *Lisää aloitusnäyttöön*.

Kamera vaatii HTTPS-yhteyden, ja GitHub Pages tarjoaa sen valmiiksi. Ensimmäisellä avauksella
ladataan noin 13 Mt (OpenCV.js), minkä jälkeen kaikki toimii offline-tilassa.

## Käyttö

1. **Kartoitus → Ota kuva.** Kuvaa kaappiseinä suoraan edestä ja kauempaa, niin että ympäristöä näkyy.
2. Valitse **Merkitse ovi** ja vedä suorakulmio oven päälle. Valitse kaappi tai luo uusi. Jos ovi on
   kuvassa vinossa, raahaa kulmapisteitä. Nipistämällä voit zoomata.
3. Lisää kaapin sisältö **Kaapit**-välilehdellä. Monta tavaraa voi liittää kerralla rivinvaihdoin.
4. Valitse **Skanneri** ja osoita kaappia. Aloita hieman kauempaa, sitten voit siirtyä lähemmäs.
5. **Haku:** kirjoita esimerkiksi "sokeri" ja paina *Näytä*. Kohdekaappi korostuu kamerakuvassa
   keltaisena, ja jos se on ruudun ulkopuolella, nuoli osoittaa sen suunnan.

**Vinkkejä:**
- Jos valaistus vaihtelee paljon (päivänvalo, iltavalot), ota samasta seinästä toinen kuva ja valitse
  editorin valikosta **Kopioi ovet**. Sovellus laskee ovien paikat uuteen kuvaan automaattisesti.
- L-keittiössä kannattaa ottaa oma kuva kustakin seinästä. Sama kaappi voi esiintyä useassa kuvassa.
- Kartoituskuvan *tunnistettavuus*-merkintä kertoo, löytyykö kuvasta tarpeeksi yksityiskohtia.
- Tähtäinkuvake oikeassa yläkulmassa avaa vianetsintänäkymän. Siinä näkyvät piirteet, osumat,
  tila, käsittelyaika ja hylkäyssyyt.
- Jos puhelin on hidas, valitse *Kartoitus → Asetukset → Tunnistuksen tarkkuus → Nopea*.
- Varmuuskopio (JSON, sisältää kuvat) löytyy asetuksista.

## Rajoitukset

- Kun kamera on aivan kiinni sileässä ovessa, sovellus ei voi tunnistaa sitä tyhjästä. Silloin
  kaappi pysyy tiedossa seurannan avulla, kunhan aloitat kauempaa.
- Jos keittiö muuttuu paljon (esim. remontti), ota uusi kartoituskuva. Tasolla liikkuvat tavarat
  eivät yleensä haittaa.

## Testit

```
npm install
npm run testdata   # tuottaa synteettisen keittiön, jossa on identtiset ovet (vaatii python3, opencv, numpy)
npm test           # tunnistus, koko seinä, oppiva kartoitus iltavalossa, reunojen kohdistus ja kaksisäikeinen putki
```

`test/e2e.py` ajaa koko sovelluksen Chromessa valekameran kanssa (Playwright), ja `test/offline.py`
tarkistaa offline-käynnistyksen. Molemmat olettavat, että kansio tarjoillaan osoitteessa
`http://localhost:8765` (`python3 -m http.server 8765`).

`test/gen_depth.py` ja `test/depth.js` mittaavat lähikuvien tarkkuutta syvyyskerroksilla. Ne
tarvitsevat oman varmuuskopion (`python3 test/gen_depth.py varmuuskopio.json "Seinä 1" kansio 0`,
sitten `node test/depth.js kansio`). Varmuuskopio ja tuotetut kuvat sisältävät kuvia kodistasi, joten
älä lisää niitä repoon.
