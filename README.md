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

Raskas tunnistus ajetaan vain tarvittaessa ja kameran ollessa suhteellisen paikallaan. Kaikki
konenäkö pyörii erillisessä säikeessä (Web Worker, OpenCV.js), joten kamerakuva pysyy sulavana.

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
npm test           # tunnistustarkkuus, liikesarja eri kuvanopeuksilla ja alueiden siirto
```

`test/e2e.py` ajaa koko sovelluksen Chromessa valekameran kanssa (Playwright), ja `test/offline.py`
tarkistaa offline-käynnistyksen. Molemmat olettavat, että kansio tarjoillaan osoitteessa
`http://localhost:8765` (`python3 -m http.server 8765`).
