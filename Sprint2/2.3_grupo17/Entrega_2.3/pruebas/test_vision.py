"""Regresiones de correspondencias, detección real y coherencia del entregable."""
import json
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
from vision import (FOTOS, RESULTADOS, crear_detector, detectar_marcadores, error_rms,
                    leer_calibracion, leer_imagen, puntos_marcador)
from comprobar import rectitud


class VisionTest(unittest.TestCase):
    def test_rms_correspondencias_con_formas_distintas(self):
        observadas = np.array([[10., 20.], [50., 80.], [200., 70.]])
        self.assertEqual(error_rms(observadas, observadas[:, None, :]), 0.)
        self.assertAlmostEqual(error_rms(observadas, (observadas + [3, 4])[:, None, :]), 5.)
        with self.assertRaises(ValueError):
            error_rms(observadas, observadas[:2])

    def test_deteccion_de_la_fotografia_de_demostracion(self):
        K, D, tamano = leer_calibracion()
        foto = leer_imagen(FOTOS / 'WIN_20260924_15_12_32_Pro.jpg', tamano)
        salida, marcadores = detectar_marcadores(foto, crear_detector(), K, D)
        self.assertEqual({m['id'] for m in marcadores}, {20, 21, 23, 47})
        self.assertEqual(salida.shape, foto.shape)
        for m in marcadores:
            self.assertTrue(np.isfinite(m['rvec']).all())
            self.assertGreater(m['tvec'][2], 0)
            self.assertLess(m['error_reproyeccion_px'], 3.)

    def test_imagen_sin_marcadores(self):
        K, D, _ = leer_calibracion()
        imagen = np.full((600, 800, 3), 255, np.uint8)
        salida, marcadores = detectar_marcadores(imagen, crear_detector(), K, D)
        self.assertEqual(marcadores, [])
        np.testing.assert_array_equal(salida, imagen)

    def test_geometria_y_unidades_del_marcador(self):
        puntos = puntos_marcador(2.)
        np.testing.assert_array_equal(puntos, [[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]])
        with self.assertRaises(ValueError):
            puntos_marcador(0)

    def test_resolucion_incompatible(self):
        with self.assertRaisesRegex(ValueError, 'Resolución incompatible'):
            leer_imagen(FOTOS / 'WIN_20260924_15_12_32_Pro.jpg', (640, 480))

    def test_rms_global_coherente_con_errores_por_foto(self):
        informe = json.loads((RESULTADOS / 'calibracion.json').read_text(encoding='utf-8'))
        K, D, tamano = leer_calibracion()
        np.testing.assert_allclose(K, informe['camera_matrix'])
        np.testing.assert_allclose(D.reshape(-1), informe['distortion_coefficients'])
        self.assertEqual(list(tamano), informe['resolucion'])
        self.assertEqual(len(informe['usadas']), informe['fotos_usadas'])
        self.assertTrue(all(f['error_rms_px'] <= 3 for f in informe['usadas']))
        rms = np.sqrt(np.mean([f['error_rms_px'] ** 2 for f in informe['usadas']]))
        self.assertAlmostEqual(rms, informe['rms_reproyeccion_px'], places=4)
        self.assertEqual(len(informe['usadas']) + len(informe['descartadas_por_error']), informe['patron_detectado'])
        self.assertEqual(informe['patron_detectado'] + len(informe['sin_patron']), informe['fotos_totales'])

    def test_rectitud_de_una_cuadricula_ideal(self):
        grid = np.mgrid[0:13, 0:9].T.reshape(-1, 2).astype(np.float32)
        self.assertLess(rectitud(grid * 15 + [100, 200]), 1e-4)


if __name__ == '__main__':
    unittest.main()
