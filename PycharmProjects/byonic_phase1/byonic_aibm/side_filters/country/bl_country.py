from byonic_aibm.side_filters.country.co_country import COMLCountry, COCountry
from byonic_aibm.side_filters.country.da_country import DACountry
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLCountry(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLCountry.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_Country: DACountry = DACountry()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLCountry.__logger

    def get_countries(self):
        countries = self.da_Country.load_all()

        if countries is not None:
            for country_key in countries.keys():
                country = countries[country_key]
                self.cache.put(country_key, country)
        return countries

    def get_countries_by_name(self):
        countries = self.da_Country.load_all()

        if countries is not None:
            for country_key in countries.keys():
                country = countries[country_key]
                self.cache.put(country_key, country)
        return countries

    def get_country(self, identifier):
        country_key = COCountry.create_key(identifier=identifier)
        country = self.cache.get(country_key)
        if country is None:
            country = self.da_Country.load_country(identifier)
            self.validate_country(country)
            self.cache.put(country.get_key(), country)
        return country

    def create_country(self, country):
        self.validate_country(country)
        self.da_Country.add_country(country)
        if country.identifier is None:
            message = COMLCountry.MSG_COUNTRY_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return country

    def update_country(self, country):
        self.validate_country(country)
        self.da_Country.update_country(country)
        self.cache.remove(country.get_key())
        return country

    def delete_country(self, identifier):
        self.da_Country.delete_country(identifier)
        country_key = COCountry.create_key(identifier=identifier)
        self.cache.remove(country_key)
        return identifier

    def save_countries(self):
        for country in self.get_countries():
            self.save_country(country)

    def save_country(self, country):
        self.validate_country(country)
        if country.mark_for_deletion:
            self.delete_country(country.identifier)
        elif country.touched:
            if country.identifier is None:
                self.create_country(country)
            else:
                self.update_country(country)
            country.touched = False

    def validate_country(self, country):
        if not isinstance(country, COCountry):
            message = COMLCountry.MSG_COUNTRY_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
