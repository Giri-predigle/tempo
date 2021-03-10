from byonic_data_access.db_connection_config import DBConnection
from byonic_aibm.usermanagement.organisation.co_organisation import COOrganization


class DAOrganisation:
    def __init__(self):
        super().__init__()

    @staticmethod
    def load_all_organisation():
        data = DBConnection().getsession()
        organisation = data.query(COOrganization.col_user_organisation).all()
        print(organisation)

    def load_organisation(self, identifier):
        data = DBConnection().getsession()
        organization = dict()

        organisation_list = data.query(COOrganization).filter(COOrganization.col_user_organisation_id == identifier).all()
        for ct in organisation_list:
            organization[ct.get_key()] = ct

        return organization


if __name__ == "__main__":
    DAOrganisation().load_all_organisation()